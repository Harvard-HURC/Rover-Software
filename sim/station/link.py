"""The station's gz-transport side: discovers the running world, spawns the
eye and chase cameras (and the fly camera when it is first wanted), keeps the
latest telemetry, publishes the driver's commands and finds other driver
stations on the same world.

Callbacks run on gz-transport threads and only store the latest message
under a lock; snapshot() hands the web side a consistent copy.
"""
import json
import math
import os
import re
import socket
import threading
import time

import gen_model
import viewers
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.double_pb2 import Double
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.image_pb2 import Image
from gz.msgs10.model_pb2 import Model
from gz.msgs10.navsat_pb2 import NavSat
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node, SubscribeOptions

from .video import Feed

ROVER = gen_model.Params()
CAMERAS = {  # page name -> image topic
    "eye": gen_model.EYE_IMAGE_TOPIC,
    "chase": gen_model.CHASE_IMAGE_TOPIC,
    "rgb": gen_model.CAMERA_TOPIC + "/image",
    "depth": gen_model.CAMERA_TOPIC + "/depth_image",
    "fly": viewers.FLY_IMAGE_TOPIC,
}
# The station's camera models, the snapshot key of their state and the state
# topic their ChaseCamera plugin publishes at 5 Hz.
VIEWERS = ((gen_model.EYE_MODEL, "eye", gen_model.EYE_STATE_TOPIC),
           (gen_model.CHASE_MODEL, "chase", gen_model.CHASE_STATE_TOPIC))
# The fly camera (FlyCamera plugin, state at 10 Hz of sim time) is spawned
# only when the Fly or Map view is first used: an idle one costs 1.5-3.5 % of
# the step time, and its first step reads the heightmap (20-310 ms).
FLY_VIEWER = (viewers.FLY_MODEL, "fly", viewers.FLY_STATE_TOPIC)
STATION_TOPIC = "/driver_station/announce"  # gz.msgs.StringMsg, JSON: id, world, url
JOINT_RATE = 20  # [Hz] JointStatePublisher sends every 1 ms step; read 20 of them a second
DRIVETRAIN_RATE = 10  # [Hz] the physical drivetrain's state comes at 50 Hz; the page shows 10
SIM_TIMEOUT = 3.0  # [s] world statistics come at 10 Hz (paused too); without them the simulation is gone
# Keys that count only while they keep coming [s]: referee data, the
# drivetrain's state (none from a DiffDrive rover), and the camera states,
# which say the camera model is in the world.
EXPIRY = {"score": 3.0, "radio": 3.0, "eye": 1.0, "chase": 1.0, "fly": 1.0, "drivetrain": 1.0}
STATION_TIMEOUT = 2.0  # [s] another driver station counts as attached this long after it announced itself
SPAWN_TIMEOUT = 5.0  # [s] for a spawned camera's plugin to report
# The binding keeps the GIL while a request waits, so gz-transport's thread
# stalls on any subscription callback and the reply can be lost although the
# server acts on the request: wait briefly, and judge by the camera's state.
REQUEST_TIMEOUT = 1000  # [ms]
# The fly camera is spawned while the station serves, with every subscription
# running: the reply is all but sure to be lost, and the wait stalls the page's
# controls, so it is short.
FLY_REQUEST_TIMEOUT = 200  # [ms]


def euler(q):
    """(roll, pitch, yaw) of a gz.msgs.Quaternion."""
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def running_worlds(node):
    """Names of the worlds whose clock is on the transport."""
    return sorted({m.group(1) for t in node.topic_list() if (m := re.fullmatch(r"/world/([^/]+)/clock", t))})


def wait_for_world(node, timeout, name=None, process=None):
    """The running world (the one called `name` if given), or None after
    timeout [s] or as soon as `process` (the gz sim that should bring it up,
    a subprocess.Popen) has exited."""
    deadline = time.monotonic() + timeout
    while True:
        worlds = running_worlds(node)
        if worlds and (name is None or name in worlds):
            return name or worlds[0]
        if time.monotonic() >= deadline or (process is not None and process.poll() is not None):
            return None
        time.sleep(0.25)


class GzLink:
    def __init__(self, world, node=None):
        self.world = world
        self.node = node or Node()
        self.id = f"{socket.gethostname()}:{os.getpid()}"
        self._lock = threading.Lock()
        self._state = {}
        self._seen = {}  # key -> time.monotonic() of the latest message
        self._stations = {}  # id of another driver station -> (its announcement, time.monotonic())
        self._yaw = (None, 0.0, 0.0)  # the latest ground truth: sim time [s], yaw, yaw rate [rad/s]
        sub = self.node.subscribe
        sub(Odometry, gen_model.GROUND_TRUTH_TOPIC, self._on_odometry)
        throttled = SubscribeOptions()
        throttled.msgs_per_sec = JOINT_RATE
        sub(Model, gen_model.JOINT_STATE_TOPIC, self._on_joints, throttled)
        sub(NavSat, gen_model.NAVSAT_TOPIC, self._on_navsat)
        sub(WorldStatistics, f"/world/{world}/stats", self._on_stats)
        sub(StringMsg, gen_model.LED_TOPIC, lambda m: self._put("led", m.data.strip().lower()))
        for _, key, topic in (*VIEWERS, FLY_VIEWER):
            sub(StringMsg, topic, lambda m, key=key: self._put(key, json.loads(m.data)))
        drivetrain = SubscribeOptions()
        drivetrain.msgs_per_sec = DRIVETRAIN_RATE
        sub(StringMsg, gen_model.DRIVETRAIN_TOPIC, lambda m: self._put("drivetrain", json.loads(m.data)), drivetrain)
        sub(StringMsg, STATION_TOPIC, self._on_station)
        sub(StringMsg, "/urc/score", lambda m: self._put("score", json.loads(m.data)))
        sub(StringMsg, "/urc/radio", lambda m: self._put("radio", m.data))
        # Advertise now: gz-transport drops messages published before the
        # subscribers have connected.
        self._twist = self.node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
        self._led = self.node.advertise(gen_model.LED_TOPIC, StringMsg)
        self._chase_cmd = self.node.advertise(gen_model.CHASE_CMD_TOPIC, Vector3d)
        self._chase_mode = self.node.advertise(gen_model.CHASE_MODE_TOPIC, StringMsg)
        self._eye = self.node.advertise(gen_model.EYE_LOOK_TOPIC, Vector3d)
        self._head = {joint: self.node.advertise(gen_model.HEAD_TOPIC.format(joint=joint), Double)
                      for joint in (gen_model.PAN_JOINT, gen_model.TILT_JOINT)}
        self._fly_cmd = self.node.advertise(viewers.FLY_CMD_TOPIC, Twist)
        self._fly_speed = self.node.advertise(viewers.FLY_SPEED_TOPIC, Double)
        self._fly_look = self.node.advertise(viewers.FLY_LOOK_TOPIC, Vector3d)
        self._fly_goto = self.node.advertise(viewers.FLY_GOTO_TOPIC, Pose)
        self._fly_mode = self.node.advertise(viewers.FLY_MODE_TOPIC, StringMsg)
        self.feeds = {name: Feed(name, topic, self._watch, self._unwatch, ROVER.camera_clip)
                      for name, topic in CAMERAS.items()}
        # Advertised by the first announce(): until then the topic in the list
        # means another station.
        self._announce = None

    # --- Inputs ------------------------------------------------------------------

    def _put(self, key, value):
        with self._lock:
            self._state[key] = value
            self._seen[key] = time.monotonic()

    def _on_odometry(self, msg):
        p, q, v = msg.pose.position, msg.pose.orientation, msg.twist.linear
        roll, pitch, yaw = euler(q)
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        # The turn rate from successive yaws: Gazebo's OdometryPublisher now and
        # then reports a yaw rate off by a multiple of 2 pi / dt (measured: one
        # message of -628 rad/s in 25 s of spinning at 0.8 rad/s).
        last_t, last_yaw, rate = self._yaw
        if last_t is not None and t > last_t:
            rate = math.remainder(yaw - last_yaw, 2 * math.pi) / (t - last_t)
        elif last_t is None or t < last_t:  # the first message, or the world was reset
            rate = 0.0
        self._yaw = (t, yaw, rate)
        self._put("pose", {"x": p.x, "y": p.y, "z": p.z, "roll": roll, "pitch": pitch, "yaw": yaw,
                           "speed": v.x, "yaw_rate": rate})

    def _on_joints(self, msg):
        self._put("joints", {j.name: (j.axis1.position, j.axis1.velocity) for j in msg.joint})

    def _on_navsat(self, msg):
        self._put("gnss", {"lat": msg.latitude_deg, "lon": msg.longitude_deg, "alt": msg.altitude})

    def _on_stats(self, msg):
        t = msg.sim_time.sec + msg.sim_time.nsec * 1e-9
        self._put("stats", {"sim_time": t, "rtf": msg.real_time_factor, "paused": msg.paused})

    def _on_station(self, msg):
        info = json.loads(msg.data)
        if info["id"] != self.id:
            with self._lock:
                self._stations[info["id"]] = (info, time.monotonic())

    def _watch(self, feed):
        self.node.subscribe(Image, feed.topic, feed.on_image)

    def _unwatch(self, feed):
        self.node.unsubscribe(feed.topic)

    def _age(self, key, now):
        return now - self._seen.get(key, -math.inf)

    def online(self):
        """Whether the world's statistics keep coming: the simulation runs (paused or not)."""
        now = time.monotonic()
        with self._lock:
            return self._age("stats", now) <= SIM_TIMEOUT

    def snapshot(self):
        """The latest of everything while the simulation runs, nothing once
        it has stopped; the EXPIRY keys only while they keep coming."""
        now = time.monotonic()
        with self._lock:
            if self._age("stats", now) > SIM_TIMEOUT:
                return {}
            return {k: v for k, v in self._state.items() if self._age(k, now) <= EXPIRY.get(k, math.inf)}

    def other_stations(self, wait=0.0):
        """Announcements of the other driver stations attached to the world.
        Before this station's first announce(), waits up to `wait` [s] for
        one while the announcement topic is advertised."""
        deadline = time.monotonic() + wait
        while True:
            now = time.monotonic()
            with self._lock:
                others = [info for info, seen in self._stations.values() if now - seen <= STATION_TIMEOUT]
            if others or now >= deadline or STATION_TOPIC not in self.node.topic_list():
                return others
            time.sleep(0.1)

    # --- Outputs -----------------------------------------------------------------

    def twist(self, vx, wz):
        msg = Twist()
        msg.linear.x, msg.angular.z = vx, wz
        self._twist.publish(msg)

    def led(self, state):
        msg = StringMsg()
        msg.data = state
        self._led.publish(msg)

    def chase(self, yaw, pitch, zoom):
        msg = Vector3d()
        msg.x, msg.y, msg.z = yaw, pitch, zoom
        self._chase_cmd.publish(msg)

    def chase_mode(self, mode):
        msg = StringMsg()
        msg.data = mode
        self._chase_mode.publish(msg)

    def eye(self, yaw, pitch):
        msg = Vector3d()
        msg.x, msg.y = yaw, pitch
        self._eye.publish(msg)

    def head(self, pan, tilt):
        for joint, angle in ((gen_model.PAN_JOINT, pan), (gen_model.TILT_JOINT, tilt)):
            msg = Double()
            msg.data = angle
            self._head[joint].publish(msg)

    def fly(self, move, turn):
        """The fly camera's command: move (forward, left, up) [cruise speeds],
        turn (yaw rate, pitch rate) [rad/s]; the plugin holds it 0.3 s."""
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = move
        msg.angular.z, msg.angular.y = turn
        self._fly_cmd.publish(msg)

    def fly_speed(self, scale):
        msg = Double()
        msg.data = scale
        self._fly_speed.publish(msg)

    def fly_look(self, yaw, pitch):
        msg = Vector3d()
        msg.x, msg.y = yaw, pitch
        self._fly_look.publish(msg)

    def fly_goto(self, view):
        """Fly the camera to drive.View `view` (a smooth flight of 0.4-1.5 s)."""
        self._fly_goto.publish(_pose(view, Pose()))

    def fly_mode(self, mode):
        msg = StringMsg()
        msg.data = mode
        self._fly_mode.publish(msg)

    def announce(self, url):
        """Tell other driver stations that this one is attached to the world."""
        if self._announce is None:
            self._announce = self.node.advertise(STATION_TOPIC, StringMsg)
        msg = StringMsg()
        msg.data = json.dumps({"id": self.id, "world": self.world, "url": url})
        self._announce.publish(msg)

    # --- World -------------------------------------------------------------------

    def ensure_cameras(self):
        """Spawn the station's camera models that are not in the world; one
        console line per camera. Blocks for up to a few seconds."""
        return [self._ensure(model, key, topic) for model, key, topic in VIEWERS]

    def ensure_fly(self, view):
        """Spawn the fly camera at drive.View `view` unless it is in the world;
        one console line. Blocks for up to a few seconds."""
        return self._ensure(*FLY_VIEWER, view, FLY_REQUEST_TIMEOUT)

    def _reports(self, key, timeout):
        """Whether the camera behind snapshot key `key` sends its state,
        waiting up to timeout [s] for it."""
        deadline = time.monotonic() + timeout
        while True:
            now = time.monotonic()
            with self._lock:
                if self._age(key, now) <= EXPIRY[key]:
                    return True
            if now >= deadline:
                return False
            time.sleep(0.05)

    def _ensure(self, model, key, topic, view=None, request_timeout=REQUEST_TIMEOUT):
        """Spawn model://<model> (at drive.View `view`, if given) unless its
        plugin already reports (the topic in the list may also be a stopped
        world's)."""
        if topic in self.node.topic_list() and self._reports(key, EXPIRY[key]):
            return f"{model}: already in the world"
        service = f"/world/{self.world}/create"
        if service not in self.node.service_list():
            return f"{model}: not spawned, the world has no {service} (UserCommands system)"
        req = EntityFactory()
        req.sdf_filename = f"model://{model}"
        req.name = model
        req.allow_renaming = False
        if view is not None:
            _pose(view, req.pose)
        ok, reply = self.node.request(service, req, EntityFactory, Boolean, request_timeout)
        if self._reports(key, SPAWN_TIMEOUT):
            return f"{model}: spawned"
        if ok and not reply.data:
            return f"{model}: not spawned, {service} refused it"
        return (f"{model}: no state from it after the spawn request; its plugin needs "
                "GZ_SIM_SYSTEM_PLUGIN_PATH to include sim/build (start the world with pixi run sim or pixi run drive)")


def _pose(view, msg):
    """Fill gz.msgs.Pose `msg` with drive.View `view`; returns msg."""
    msg.position.x, msg.position.y, msg.position.z = view.x, view.y, view.z
    msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = view.quaternion()
    return msg
