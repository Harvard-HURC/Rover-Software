#!/usr/bin/env python3
"""Referee for the URC mission worlds: pixi run referee <mission> [options].

Run it next to the simulation (pixi run sim urc_<mission>). It reads the
mission sheet sim/worlds/urc_<mission>.json and, until the mission time runs
out or Ctrl-C:

- judges the tasks (sim/urc/judge.py) and prints each event;
- scripts the astronaut (Autonomy) and gives its commands;
- shows the rover's status LED: the rover's software publishes "red",
  "blue", "green" or "off" on /model/rover/led (ROS: /led), the referee
  colours the LED on the rover (green flashes);
- publishes /urc/score (JSON), /urc/radio ("los"/"nlos" between the C2
  antenna and the rover), and for Equipment Servicing /urc/launch_key and
  /urc/display (the e-paper display's text).
"""
import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM_DIR))
import gen_model  # noqa: E402
from urc import judge as J  # noqa: E402
from urc import props, rules, sheet as sheets  # noqa: E402

LED_COLORS = {"red": (1.0, 0.05, 0.05), "blue": (0.1, 0.3, 1.0), "green": (0.1, 1.0, 0.2), "off": (0.15, 0.15, 0.15)}
RATE = 20.0  # [Hz] referee ticks (wall clock)
STRING_TOPICS = ("/astronaut/command", "/astronaut/speech", "/urc/score", "/urc/radio", "/urc/display",
                 "/urc/launch_key")
LATCHED = ("/urc/display", "/urc/launch_key")  # re-sent every second for late subscribers


class Referee:
    def __init__(self, mission, sheet_path, judge, node=None):
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.material_color_pb2 import MaterialColor
        from gz.msgs10.model_pb2 import Model
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.transport13 import Node
        self.msgs = {"Double": Double, "MaterialColor": MaterialColor, "StringMsg": StringMsg}
        self.mission = mission
        self.sheet = sheets.load(sheet_path)
        self.terrain = sheets.terrain(self.sheet, sheet_path)
        self.judge = judge
        self.world = self.sheet["world"]
        self.node = node or Node()
        self.lock = threading.Lock()
        self.t = None
        self.rover = None
        self.led = "off"
        self.poses, self.joints, self.presses = {}, {}, []
        self.node.subscribe(Odometry, gen_model.GROUND_TRUTH_TOPIC, self._on_odometry)
        self.node.subscribe(StringMsg, gen_model.LED_TOPIC, self._on_led)
        self.node.subscribe(StringMsg, "/model/lander/presses", self._on_press)
        for name in set(self.sheet["objects"]) | {"lander"}:
            self.node.subscribe(Model, f"/model/{name}/state", lambda m, n=name: self._on_state(n, m))
        # Advertise everything now: gz-transport drops what is published before
        # subscribers (the ROS bridge, the rover) have connected to a topic.
        self.pubs = {}
        for topic in STRING_TOPICS:
            self._pub(topic, "StringMsg")
        self._pub(f"/world/{self.world}/material_color", "MaterialColor")
        for joint in props.ASTRONAUT_JOINTS:
            self._pub(f"/model/astronaut/joint/{joint}/0/cmd_pos", "Double")
        self.latched = {}
        self.last_led = None
        self.printed = 0
        self.last_publish = -1.0
        self.radio = None

    # --- Inputs ------------------------------------------------------------------

    def _on_odometry(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            self.rover = (p.x, p.y, p.z, yaw)

    def _on_led(self, msg):
        with self.lock:
            self.led = msg.data.strip().lower() if msg.data.strip().lower() in rules.LED_STATES else "off"

    def _on_press(self, msg):
        with self.lock:
            self.presses.append(msg.data)

    def _on_state(self, name, msg):
        with self.lock:
            for k, link in enumerate(msg.link):
                p, q = link.pose.position, link.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                pose = (p.x, p.y, p.z, 0.0, 0.0, yaw)
                self.poses[f"{name}::{link.name}"] = pose
                if k == 0:
                    self.poses[name] = pose
            for joint in msg.joint:
                self.joints[f"{name}::{joint.name}"] = joint.axis1.position

    # --- Outputs -----------------------------------------------------------------

    def _pub(self, topic, kind):
        if topic not in self.pubs:
            self.pubs[topic] = self.node.advertise(topic, self.msgs[kind])
        return self.pubs[topic]

    def say(self, topic, text):
        if topic in LATCHED:
            self.latched[topic] = text
        msg = self.msgs["StringMsg"]()
        msg.data = text
        self._pub(topic, "StringMsg").publish(msg)

    def set_pose(self, model, pose):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_pb2 import Pose
        x, y, z, yaw = pose
        req = Pose()
        req.name = model
        req.position.x, req.position.y, req.position.z = x, y, z
        req.orientation.w, req.orientation.z = math.cos(yaw / 2), math.sin(yaw / 2)
        self.node.request(f"/world/{self.world}/set_pose", req, Pose, Boolean, 200)

    def joint_targets(self, model, targets):
        for joint, angle in targets.items():
            msg = self.msgs["Double"]()
            msg.data = angle
            self._pub(f"/model/{model}/joint/{joint}/0/cmd_pos", "Double").publish(msg)

    def show_led(self, state, t, force=False):
        on = state != "green" or int(t * 4) % 2 == 0  # green flashes at 2 Hz
        shown = state if on else "off"
        if shown == self.last_led and not force:
            return
        self.last_led = shown
        msg = self.msgs["MaterialColor"]()
        msg.entity.name = f"rover::base_link::{gen_model.LED_VISUAL}"
        r, g, b = LED_COLORS[shown]
        for color in (msg.ambient, msg.diffuse):
            color.r, color.g, color.b, color.a = r, g, b, 1.0
        e = 0.0 if shown == "off" else 1.0
        msg.emissive.r, msg.emissive.g, msg.emissive.b, msg.emissive.a = r * e, g * e, b * e, 1.0
        self._pub(f"/world/{self.world}/material_color", "MaterialColor").publish(msg)

    # --- Loop ------------------------------------------------------------------

    def tick(self):
        with self.lock:
            if self.rover is None:
                return False
            obs = J.Observation(self.t, self.rover, self.led, dict(self.poses), dict(self.joints), self.presses)
            self.presses = []
        for action in self.judge.step(obs):
            if isinstance(action, J.SetPose):
                self.set_pose(action.model, action.pose)
            elif isinstance(action, J.JointTargets):
                self.joint_targets(action.model, action.targets)
            elif isinstance(action, J.Say):
                self.say(action.topic, action.text)
        if obs.t < self.last_publish:  # the world was reset
            self.last_publish = -1.0
        every_second = obs.t - self.last_publish >= 1.0
        self.show_led(obs.led, obs.t, force=every_second)
        for t, text in self.judge.events[self.printed:]:
            print(f"[{t:7.1f} s] {text}", flush=True)
        self.printed = len(self.judge.events)
        if every_second:
            self.last_publish = obs.t
            for topic, text in self.latched.items():
                self.say(topic, text)
            self.say("/urc/score", json.dumps(self.judge.summary()))
            a = self.sheet["c2"]["antenna"]
            los = sheets.radio_los(self.terrain, (a["x"], a["y"], a["z"]), *obs.rover[:3])
            self.say("/urc/radio", "los" if los else "nlos")
            if los != self.radio:
                if self.radio is not None:
                    print(f"[{obs.t:7.1f} s] radio: {'line of sight' if los else 'NO line of sight to C2'}",
                          flush=True)
                self.radio = los
        return True

    def run(self, duration=None):
        limit = self.sheet.get("time_limit_s")
        limit = max(limit) if isinstance(limit, list) else limit
        start = time.time()
        waited = False
        try:
            while duration is None or time.time() - start < duration:
                if not self.tick() and not waited:
                    print("waiting for the simulation (is it running and unpaused?)", flush=True)
                    waited = True
                if limit and self.t is not None and self.t >= limit:
                    print(f"mission time ({limit / 60:.0f} min) is up", flush=True)
                    break
                time.sleep(1 / RATE)
        except KeyboardInterrupt:
            pass
        return self.judge.summary()


def main():
    from urc.missions import MISSIONS
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("mission", choices=sorted(MISSIONS))
    parser.add_argument("--method", choices=J.COMMAND_METHODS, default="device",
                        help="how the astronaut gives commands (Autonomy; Follow! scores by method)")
    parser.add_argument("--key", help="launch key to type (Equipment Servicing; default: random)")
    parser.add_argument("--seed", type=int, help="seed for the random launch key")
    args = parser.parse_args()
    sheet_path = sheets.path(f"urc_{args.mission}")
    sheet = sheets.load(sheet_path)
    key = args.key or (J.launch_key(args.seed) if args.mission == "equipment_servicing" else None)
    judge = J.make_judge(args.mission, sheet, sheet_path, method=args.method, key=key)
    print(f"URC 2027 {sheet['mission']} referee: world {sheet['world']}, Ctrl-C to stop", flush=True)
    summary = Referee(args.mission, sheet_path, judge).run()
    print(json.dumps({k: v for k, v in summary.items() if k != "events"}, indent=2))


if __name__ == "__main__":
    main()
