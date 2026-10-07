"""Run the rover headless in Gazebo for the tests.

Gazebo's Python TestFixture runs the server inside this process, so a run is
repeatable: commands go out over gz-transport every 20 ms of sim time, and
the state is read from the entity-component manager after the last step.
"""
import contextlib
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

from worldfiles import SIM_DIR, temp_sdf

import gen_model  # noqa: E402  (worldfiles puts sim/ on the path)

# Set before Gazebo starts: where model://rover and the plugin live, and a
# private transport partition so tests never talk to a running simulation.
os.environ["GZ_SIM_RESOURCE_PATH"] = str(SIM_DIR / "models")
os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(SIM_DIR / "build")
os.environ["GZ_PARTITION"] = f"rover_sim_test_{os.getpid()}"
os.environ.setdefault("GZ_IP", "127.0.0.1")
# The conda gz-rendering has a space-padded OGRE plugin path; point it at the
# real one so worlds with cameras can render (sim/README.md, Troubleshooting).
os.environ.setdefault("OGRE2_RESOURCE_PATH", str(Path(sys.prefix) / "lib" / "OGRE-Next"))

import gz.math7  # noqa: E402,F401  (lets gz.sim8 return Pose3d values)
from gz.msgs10.twist_pb2 import Twist  # noqa: E402
from gz.sim8 import Joint, Link, Model, TestFixture, World, world_entity  # noqa: E402
from gz.transport13 import Node  # noqa: E402

LINKS = ("base_link", "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr")
ROCKERS = ("rocker_left_joint", "rocker_right_joint")


@dataclass
class State:
    """The rover after the last step."""

    rockers: dict  # joint name -> angle [rad]
    poses: dict  # link name -> (x, y, z, roll, pitch, yaw) in the world
    rocker_peak: float = 0.0  # the largest |rocker angle| during the run [rad]
    messages: dict = field(default_factory=dict)  # topic -> messages received


def world_sdf(extra="", spawn_z=0.02):
    """Flat ground (DART, 1 ms steps), the rover at the origin, plus `extra` SDF."""
    return f"""<?xml version="1.0"?>
<sdf version="1.11">
  <world name="test">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
      </link>
    </model>
    {extra}
    <include><uri>model://rover</uri><pose>0 0 {spawn_z} 0 0 0</pose></include>
  </world>
</sdf>
"""


def simulate(seconds, extra="", spawn_z=0.02, cmd=(0.0, 0.0), subscribe=(), world=None):
    """Run for `seconds` of sim time and return the final State.

    cmd: (vx [m/s], wz [rad/s]) for DiffDrive, sent from the start.
    subscribe: (topic, message class) pairs to record during the run.
    world: an SDF file to run instead of world_sdf(extra, spawn_z).
    """
    with temp_sdf(world_sdf(extra, spawn_z)) if world is None else contextlib.nullcontext(str(world)) as world_path:
        return _run(seconds, cmd, subscribe, world_path)


def _run(seconds, cmd, subscribe, world_path):
    node = Node()
    messages = {topic: [] for topic, _ in subscribe}
    for topic, msg_type in subscribe:
        node.subscribe(msg_type, topic, messages[topic].append)
    publisher = node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
    twist = Twist()
    twist.linear.x, twist.angular.z = cmd
    handles = {}
    last = {}
    peak = [0.0]

    def pre_update(info, ecm):
        if not handles:
            model = Model(World(world_entity(ecm)).model_by_name(ecm, "rover"))
            handles["rockers"] = {n: Joint(model.joint_by_name(ecm, n)) for n in ROCKERS}
            handles["links"] = {n: Link(model.link_by_name(ecm, n)) for n in LINKS}
            for joint in handles["rockers"].values():
                joint.enable_position_check(ecm, True)
        if any(cmd) and info.iterations % 20 == 0:
            publisher.publish(twist)

    def post_update(info, ecm):
        last["rockers"] = {n: j.position(ecm) for n, j in handles["rockers"].items()}
        last["poses"] = {n: link.world_pose(ecm) for n, link in handles["links"].items()}
        peak[0] = max([peak[0]] + [abs(q[0]) for q in last["rockers"].values() if q])

    fixture = TestFixture(world_path)
    fixture.on_pre_update(pre_update)
    fixture.on_post_update(post_update)
    fixture.finalize()
    fixture.server().run(True, round(seconds * 1000), False)
    if not last:
        raise RuntimeError(f"Gazebo did not step {world_path}; see the [Err] lines above")
    rockers = {n: q[0] for n, q in last["rockers"].items()}
    return State(rockers, {n: _xyzrpy(pose) for n, pose in last["poses"].items()}, peak[0], messages)


def _xyzrpy(pose):
    p, r = pose.pos(), pose.rot().euler()
    return (p.x(), p.y(), p.z(), r.x(), r.y(), r.z())
