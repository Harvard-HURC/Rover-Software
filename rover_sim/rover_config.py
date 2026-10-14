"""The rover's numbers for the simulation's Python, read once: rover_description/config/rover.yaml, the physical
rover (ROVER), and rover_sim/config/sim.yaml, how the simulation models it (SIM), as read-only trees with attribute
access (ROVER.wheel.radius, SIM.wheel_cmd.accel; YAML lists become tuples). Variant is a rover with some of those
numbers changed, which tools/gen_rover.py builds (the tests drive such variants). The names of the rover's Gazebo
interface (topics, joints, files in its model) are here too.

The station, referee, judge, viewers, world generator and tests read the rover here; nothing else reads the two
files but the xacro (urdf/rover_sim.urdf.xacro).
"""
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

import yaml

SIM_DIR = Path(__file__).resolve().parent
DESCRIPTION_DIR = SIM_DIR.parent / "rover_description"  # the package beside this one in the repository
ROVER_YAML = DESCRIPTION_DIR / "config" / "rover.yaml"
SIM_YAML = SIM_DIR / "config" / "sim.yaml"
CONTROL_DIR = SIM_DIR.parent / "rover_control"  # the package beside this one: the controllers the ROS rover runs
CONTROLLERS_YAML = CONTROL_DIR / "config" / "controllers.yaml"


class Tree(Mapping):
    """A read-only YAML mapping: tree.key or tree["key"]; nested mappings are Trees, lists tuples."""

    def __init__(self, data):
        self._data = {key: _freeze(value) for key, value in data.items()}

    def __getitem__(self, key):
        return self._data[key]

    def __getattr__(self, name):
        try:
            return self._data[name]
        except KeyError:
            raise AttributeError(name) from None

    def __iter__(self):
        return iter(self._data)

    def __len__(self):
        return len(self._data)

    def __hash__(self):
        return hash(tuple(self._data.items()))

    def __eq__(self, other):
        return isinstance(other, Tree) and self._data == other._data

    def __repr__(self):
        return f"Tree({self.plain()!r})"

    def merged(self, changes):
        """A copy with `changes` (a nested dict) merged in: a mapping merges into the mapping it replaces, anything
        else replaces the value. Every key must exist already (KeyError: a misspelt name never adds a number the
        model ignores)."""
        data = dict(self._data)
        for key, value in changes.items():
            if key not in data:
                raise KeyError(f"no {key!r} among {sorted(data)}")
            old = data[key]
            data[key] = old.merged(value) if isinstance(old, Tree) and isinstance(value, Mapping) else value
        return Tree(data)

    def plain(self):
        """Plain dicts and lists, for yaml.safe_dump."""
        return {key: value.plain() if isinstance(value, Tree) else _plain(value) for key, value in self._data.items()}


def _freeze(value):
    if isinstance(value, Mapping):
        return value if isinstance(value, Tree) else Tree(value)
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(v) for v in value)
    return value


def _plain(value):
    if isinstance(value, Tree):
        return value.plain()
    if isinstance(value, tuple):
        return [_plain(v) for v in value]
    return value


def load(path):
    """A YAML file as a Tree."""
    return Tree(yaml.safe_load(Path(path).read_text()))


ROVER = load(ROVER_YAML)
SIM = load(SIM_YAML)


@dataclass(frozen=True)
class Variant:
    """A rover to build (gen_rover.build_sdf): rover.yaml's numbers and sim.yaml's, the files' own by default."""
    rover: Tree = field(default=ROVER)
    sim: Tree = field(default=SIM)

    def replace(self, **sections):
        """A copy with sections changed, each a top-level key of rover.yaml or of sim.yaml (they share none) and a
        nested dict of new values, or a new value: replace(contact={"dig": False}, motor={"current_limit": 10.0},
        tread_tyre=False)."""
        rover = {k: v for k, v in sections.items() if k in self.rover}
        sim = {k: v for k, v in sections.items() if k not in self.rover}
        return Variant(self.rover.merged(rover), self.sim.merged(sim))


# The rover's Gazebo interface (sim.yaml topics; the joint and visual names are the description's).
CMD_VEL_TOPIC = SIM.topics.cmd_vel
WHEEL_CMD_TOPIC = SIM.topics.wheel_cmd  # the drivetrain's wheel setpoints: gz.msgs.Double_V, fl rl fr rr (wheel_cmd.py)
ODOM_TOPIC = SIM.topics.odometry
TF_TOPIC = SIM.topics.tf
JOINT_STATE_TOPIC = SIM.topics.joint_states
GROUND_TRUTH_TOPIC = SIM.topics.ground_truth
IMU_TOPIC = SIM.topics.imu
NAVSAT_TOPIC = SIM.topics.navsat
CAMERA_TOPIC = SIM.topics.camera  # rgbd: /image, /depth_image, /points, /camera_info
LED_TOPIC = SIM.topics.led  # set by the rover's software, shown by referee.py
DRIVETRAIN_TOPIC = SIM.topics.drivetrain  # physical drivetrain: gz.msgs.StringMsg, JSON (design spec 9.3)
DUST_TOPIC = SIM.topics.dust  # .format(link=..., emitter=...): gz.msgs.ParticleEmitter
HEAD_TOPIC = SIM.topics.head  # .format(joint=...): gz.msgs.Double target angle [rad]
PAN_JOINT = "camera_pan_joint"
TILT_JOINT = "camera_tilt_joint"
LED_VISUAL = "led_visual"  # on base_link
WHEEL_MESH = "meshes/wheel.glb"  # in the rover model: a copy of rover_description/meshes/wheel.glb
DUST_SPRITE = "materials/textures/dust_puff.png"  # in the rover model: a soft dust puff (textures.dust_puff)
SIDES = (("left", "l", 1.0), ("right", "r", -1.0))  # name, suffix, sign of y
ENDS = (("front", "f", 1.0), ("rear", "r", -1.0))  # name, prefix, sign of x
