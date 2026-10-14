# Gazebo Rover Model Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A generic Gazebo Harmonic model of the 4-wheel rocker-differential tank rover, drivable over ROS 2, with headless tests.

**Architecture:** A stdlib-only Python script (`sim/gen_model.py`) turns one `Params` dataclass into `sim/models/rover/model.sdf`. A small C++ gz-sim system plugin (`RockerDifferential`) couples the rockers because DART, the engine we need for skid-steer friction, has no mimic joints. Tests run Gazebo in-process via the `gz.sim8` Python `TestFixture`.

**Tech Stack:** Gazebo Harmonic (gz-sim 8.10, gz-physics 7 / DART 6.19, sdformat 14), Python 3.12 `unittest`, C++17 + CMake, ROS 2 Jazzy `ros_gz_bridge`, all from the pixi env.

**Spec:** `docs/superpowers/specs/2026-10-05-gazebo-rover-model-design.md`

**No git:** the workspace is not a git repository, so there are no commit steps.

**Environment notes (learned while prototyping):**
- Run everything through pixi (`pixi run …`): `gz`, `cmake`, Python bindings live in the env.
- `import gz.math7` before reading poses from `gz.sim8`, or pybind11 cannot convert `Pose3d`.
- gz-transport may log `Exception sending a multicast message: No route to host` in sandboxes; harmless. The tests set `GZ_IP=127.0.0.1`.
- Python gz-sim system plugins crash the Python `TestFixture` (embedded interpreter); that is why the differential is C++.

## File map

| File | Responsibility |
|---|---|
| `pixi.toml` (modify) | `sim-build`, `sim-model`, `sim`, `sim-bridge`, `sim-test` tasks |
| `.gitignore` (modify) | ignore `sim/build/` |
| `sim/CMakeLists.txt` | build the plugin |
| `sim/plugins/rocker_differential.cpp` | the differential system plugin |
| `sim/gen_model.py` | `Params`, inertia helpers, `build_sdf()`, topic names, `main()` |
| `sim/models/rover/model.config` | Gazebo model manifest |
| `sim/models/rover/model.sdf` | generated |
| `sim/worlds/rover_test.sdf` | demo world |
| `sim/bridge.yaml` | ROS 2 bridge config |
| `sim/run.sh` | start server + GUI (split on macOS) |
| `sim/tests/test_gen_model.py` | generator unit tests |
| `sim/tests/simulate.py` | headless-sim helper |
| `sim/tests/test_rover_sim.py` | physics tests |
| `sim/README.md` | usage + design notes |

---

### Task 1: Scaffold and pixi tasks

**Files:**
- Modify: `pixi.toml` (`[tasks]`)
- Modify: `.gitignore`

- [ ] **Step 1: Add tasks to `pixi.toml`** under the existing `[tasks]` entries:

```toml
sim-build = "cmake -S sim -B sim/build -DCMAKE_BUILD_TYPE=Release && cmake --build sim/build -j"
sim-model = "python sim/gen_model.py"
sim = { cmd = "bash sim/run.sh", depends-on = ["sim-build", "sim-model"] }
sim-bridge = "ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=sim/bridge.yaml"
sim-test = { cmd = "python -m unittest discover -s sim/tests -v", depends-on = ["sim-build", "sim-model"] }
```

- [ ] **Step 2: Ignore the plugin build** — append to `.gitignore`:

```
# sim plugin build output
sim/build/
```

- [ ] **Step 3: Create directories**

Run: `mkdir -p sim/plugins sim/models/rover sim/worlds sim/tests`

---

### Task 2: Inertia helpers (TDD)

**Files:**
- Create: `sim/tests/test_gen_model.py`
- Create: `sim/gen_model.py`

- [ ] **Step 1: Write the failing tests** — `sim/tests/test_gen_model.py`:

```python
#!/usr/bin/env python3
"""Unit tests for sim/gen_model.py (no physics; pixi run sim-test)."""
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import gen_model  # noqa: E402

P = gen_model.Params()


class Inertia(unittest.TestCase):
    def test_box(self):
        self.assertEqual(gen_model.box_inertia(12.0, (1.0, 2.0, 3.0)), (13.0, 10.0, 5.0))

    def test_wheel_axis_is_y(self):
        ixx, iyy, izz = gen_model.wheel_inertia(2.0, 0.5, 1.0)
        self.assertAlmostEqual(iyy, 2.0 * 0.5**2 / 2)
        self.assertAlmostEqual(ixx, 2.0 * (3 * 0.5**2 + 1.0**2) / 12)
        self.assertEqual(ixx, izz)

    def test_flat_rocker_is_one_rod(self):
        # Rods to (+-1, 0, 0) form one rod of length 2 about its middle.
        self.assertEqual(gen_model.rocker_inertia(2.0, 1.0, 0.0), (0.0, 2.0 * 4 / 12, 2.0 * 4 / 12))

    def test_vertical_rocker_is_one_rod(self):
        # Both rods straight down to (0, 0, -1) overlap: one rod of length 1.
        ixx, iyy, izz = gen_model.rocker_inertia(2.0, 0.0, -1.0)
        self.assertAlmostEqual(ixx, 2.0 / 12)
        self.assertAlmostEqual(iyy, 2.0 / 12)
        self.assertEqual(izz, 0.0)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run to verify failure**

Run: `pixi run python -m unittest sim/tests/test_gen_model.py -v`
Expected: ERROR — `ModuleNotFoundError: No module named 'gen_model'`.

- [ ] **Step 3: Implement** — `sim/gen_model.py` with the module docstring, `Params`, and the helpers (the rest of the file comes in Task 3):

```python
#!/usr/bin/env python3
"""Generate the rover's Gazebo model, sim/models/rover/model.sdf.

Every dimension lives in Params. The defaults are the placeholder geometry
from driver/include/rover_driver/config.hpp until the mechanical team has
real numbers: edit Params, then run `pixi run sim-model`.

Frames follow the driver: x forward, y left, z up, origin on the ground
midway between the four wheels with the rockers at zero.
"""
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

MODEL_DIR = Path(__file__).resolve().parent / "models" / "rover"

CMD_VEL_TOPIC = "/model/rover/cmd_vel"
ODOM_TOPIC = "/model/rover/odometry"
TF_TOPIC = "/model/rover/tf"
JOINT_STATE_TOPIC = "/model/rover/joint_states"
GROUND_TRUTH_TOPIC = "/model/rover/ground_truth"
IMU_TOPIC = "/model/rover/imu"


@dataclass(frozen=True)
class Params:
    wheel_radius: float = 0.15
    wheel_width: float = 0.10
    # Rocker pivots at (0, +-pivot_y, pivot_z).
    pivot_y: float = 0.40
    pivot_z: float = 0.35
    # Wheel centers at pivot + (+-wheel_dx, 0, wheel_dz): front +, rear -.
    wheel_dx: float = 0.45
    wheel_dz: float = -0.20
    chassis_size: tuple[float, float, float] = (0.80, 0.50, 0.20)
    chassis_z: float = 0.40  # height of the chassis box center
    chassis_mass: float = 30.0
    rocker_mass: float = 3.0
    wheel_mass: float = 2.5
    rocker_limit: float = 0.5  # [rad] each way
    rocker_damping: float = 0.5  # [N m s/rad]
    # Differential spring on q_L + q_R; see plugins/rocker_differential.cpp.
    diff_stiffness: float = 5000.0  # [N m/rad]
    diff_damping: float = 100.0  # [N m s/rad]
    wheel_effort: float = 30.0  # [N m] motor torque limit
    wheel_speed: float = 10.0  # [rad/s]
    # Tire friction. Lateral below longitudinal emulates tire scrub; with equal
    # values the physics engine's box friction stops the rover turning in place.
    mu_longitudinal: float = 1.0
    mu_lateral: float = 0.5
    imu_rate: float = 100.0  # [Hz]
    gyro_noise: float = 0.002  # [rad/s] stddev
    accel_noise: float = 0.02  # [m/s^2] stddev
    # Rocker arm bars, visual and collision only (their mass is rocker_mass).
    arm_inset: float = 0.08  # from the wheel center plane toward the chassis
    arm_thickness: float = 0.04
    arm_height: float = 0.05


def box_inertia(mass, size):
    """Principal moments (ixx, iyy, izz) of a solid box about its center."""
    x, y, z = size
    return (mass * (y * y + z * z) / 12, mass * (x * x + z * z) / 12, mass * (x * x + y * y) / 12)


def wheel_inertia(mass, radius, width):
    """Principal moments of a solid cylinder whose axis is y."""
    across = mass * (3 * radius * radius + width * width) / 12
    return (across, mass * radius * radius / 2, across)


def rocker_inertia(mass, dx, dz):
    """Principal moments of a rocker: two thin rods of mass / 2 from the pivot
    to (+-dx, 0, dz), about their joint center of mass (0, 0, dz / 2)."""
    return (mass * dz * dz / 12, mass * (dx * dx + dz * dz) / 12 + mass * dx * dx / 4, mass * dx * dx / 3)
```

- [ ] **Step 4: Run to verify pass**

Run: `pixi run python -m unittest sim/tests/test_gen_model.py -v`
Expected: 4 tests OK.

---

### Task 3: SDF generation (TDD)

**Files:**
- Modify: `sim/tests/test_gen_model.py`
- Modify: `sim/gen_model.py`
- Create: `sim/models/rover/model.config`
- Generate: `sim/models/rover/model.sdf`

- [ ] **Step 1: Write the failing tests** — add before the `if __name__` block:

```python
def vec(text):
    return [float(v) for v in text.split()]


class Structure(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdf = gen_model.build_sdf(P)
        cls.model = ET.fromstring(cls.sdf).find("model")

    def plugin(self, name):
        return self.model.find(f"plugin[@name='{name}']")

    def test_joint_tree(self):
        tree = {j.get("name"): (j.findtext("parent"), j.findtext("child")) for j in self.model.findall("joint")}
        self.assertEqual(tree, {
            "rocker_left_joint": ("base_link", "rocker_left"),
            "rocker_right_joint": ("base_link", "rocker_right"),
            "wheel_fl_joint": ("rocker_left", "wheel_fl"),
            "wheel_rl_joint": ("rocker_left", "wheel_rl"),
            "wheel_fr_joint": ("rocker_right", "wheel_fr"),
            "wheel_rr_joint": ("rocker_right", "wheel_rr"),
        })
        for joint in self.model.findall("joint"):
            self.assertEqual(vec(joint.findtext("axis/xyz")), [0, 1, 0], joint.get("name"))

    def test_wheel_centers_follow_params(self):
        z = P.pivot_z + P.wheel_dz
        expected = {"wheel_fl": (P.wheel_dx, P.pivot_y), "wheel_rl": (-P.wheel_dx, P.pivot_y),
                    "wheel_fr": (P.wheel_dx, -P.pivot_y), "wheel_rr": (-P.wheel_dx, -P.pivot_y)}
        for name, (x, y) in expected.items():
            pose = vec(self.model.find(f"link[@name='{name}']/pose").text)
            for got, want in zip(pose, (x, y, z, 0, 0, 0)):
                self.assertAlmostEqual(got, want, msg=name)

    def test_wheels_rest_on_the_ground(self):
        self.assertAlmostEqual(P.pivot_z + P.wheel_dz, P.wheel_radius)

    def test_total_mass(self):
        total = sum(float(m.text) for m in self.model.iter("mass"))
        self.assertAlmostEqual(total, P.chassis_mass + 2 * P.rocker_mass + 4 * P.wheel_mass)

    def test_drive_is_tank(self):
        drive = self.plugin("gz::sim::systems::DiffDrive")
        self.assertEqual([e.text for e in drive.findall("left_joint")], ["wheel_fl_joint", "wheel_rl_joint"])
        self.assertEqual([e.text for e in drive.findall("right_joint")], ["wheel_fr_joint", "wheel_rr_joint"])
        self.assertAlmostEqual(float(drive.findtext("wheel_separation")), 2 * P.pivot_y)
        self.assertAlmostEqual(float(drive.findtext("wheel_radius")), P.wheel_radius)
        self.assertEqual(drive.findtext("topic"), gen_model.CMD_VEL_TOPIC)

    def test_differential_couples_the_rockers(self):
        diff = self.plugin("rover_sim::RockerDifferential")
        self.assertEqual(diff.get("filename"), "RockerDifferential")
        self.assertEqual(diff.findtext("left_joint"), "rocker_left_joint")
        self.assertEqual(diff.findtext("right_joint"), "rocker_right_joint")
        self.assertAlmostEqual(float(diff.findtext("stiffness")), P.diff_stiffness)
        self.assertIsNone(self.model.find(".//mimic"))

    def test_tire_friction_axes(self):
        for ode in self.model.iter("ode"):
            self.assertEqual(vec(ode.findtext("fdir1")), [0, 0, 1])  # the axle
            self.assertAlmostEqual(float(ode.findtext("mu")), P.mu_lateral)
            self.assertAlmostEqual(float(ode.findtext("mu2")), P.mu_longitudinal)
        self.assertEqual(len(list(self.model.iter("ode"))), 4)

    def test_gz_accepts_it(self):
        with tempfile.NamedTemporaryFile("w", suffix=".sdf") as f:
            f.write(self.sdf)
            f.flush()
            result = subprocess.run(["gz", "sdf", "-k", f.name], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Valid", result.stdout)
```

- [ ] **Step 2: Run to verify failure**

Run: `pixi run python -m unittest sim/tests/test_gen_model.py -v`
Expected: Structure tests ERROR — `AttributeError: module 'gen_model' has no attribute 'build_sdf'`.

- [ ] **Step 3: Implement** — append to `sim/gen_model.py`:

```python
CHASSIS_COLOR = (0.85, 0.55, 0.15, 1)
ROCKER_COLOR = (0.35, 0.35, 0.38, 1)
TIRE_COLOR = (0.1, 0.1, 0.1, 1)
SIDES = (("left", "l", 1.0), ("right", "r", -1.0))  # name, suffix, sign of y
ENDS = (("front", "f", 1.0), ("rear", "r", -1.0))  # name, prefix, sign of x


def build_sdf(p: Params) -> str:
    """The rover model as an SDF 1.11 document."""
    sdf = ET.Element("sdf", version="1.11")
    model = _sub(sdf, "model", name="rover")
    _add_chassis(model, p)
    for side, s, sign in SIDES:
        _add_rocker(model, p, side, sign)
        for _, e, ahead in ENDS:
            _add_wheel(model, p, side, f"wheel_{e}{s}", sign, ahead)
    _add_plugins(model, p)
    ET.indent(sdf)
    return '<?xml version="1.0"?>\n' + ET.tostring(sdf, encoding="unicode") + "\n"


def _fmt(value):
    if isinstance(value, (tuple, list)):
        return " ".join(_fmt(v) for v in value)
    return f"{value:.9g}"


def _sub(parent, tag, text=None, **attrib):
    element = ET.SubElement(parent, tag, {k: str(v) for k, v in attrib.items()})
    if text is not None:
        element.text = text if isinstance(text, str) else _fmt(text)
    return element


def _inertial(link, mass, moments, xyz=(0, 0, 0)):
    inertial = _sub(link, "inertial")
    _sub(inertial, "pose", (*xyz, 0, 0, 0))
    _sub(inertial, "mass", mass)
    inertia = _sub(inertial, "inertia")
    for key, value in zip(("ixx", "iyy", "izz"), moments):
        _sub(inertia, key, value)
    for key in ("ixy", "ixz", "iyz"):
        _sub(inertia, key, 0.0)


def _box(size):
    return lambda geometry: _sub(_sub(geometry, "box"), "size", size)


def _cylinder(radius, length):
    def build(geometry):
        cylinder = _sub(geometry, "cylinder")
        _sub(cylinder, "radius", radius)
        _sub(cylinder, "length", length)

    return build


def _shape(link, name, geometry, pose, color, surface=None):
    """A collision and a matching visual."""
    for kind in ("collision", "visual"):
        element = _sub(link, kind, name=f"{name}_{kind}")
        _sub(element, "pose", pose)
        geometry(_sub(element, "geometry"))
        if kind == "visual":
            material = _sub(element, "material")
            _sub(material, "ambient", color)
            _sub(material, "diffuse", color)
        elif surface is not None:
            surface(_sub(element, "surface"))


def _tire_surface(p):
    def build(surface):
        ode = _sub(_sub(surface, "friction"), "ode")
        # fdir1 is the axle in the collision frame (the cylinder's z axis):
        # mu acts along it (lateral), mu2 across it (longitudinal).
        _sub(ode, "mu", p.mu_lateral)
        _sub(ode, "mu2", p.mu_longitudinal)
        _sub(ode, "fdir1", (0, 0, 1))

    return build


def _revolute(model, name, parent, child, lower, upper, effort=None, velocity=None, damping=None):
    joint = _sub(model, "joint", name=name, type="revolute")
    _sub(joint, "parent", parent)
    _sub(joint, "child", child)
    axis = _sub(joint, "axis")
    _sub(axis, "xyz", (0, 1, 0))
    limit = _sub(axis, "limit")
    _sub(limit, "lower", lower)
    _sub(limit, "upper", upper)
    if effort is not None:
        _sub(limit, "effort", effort)
    if velocity is not None:
        _sub(limit, "velocity", velocity)
    if damping is not None:
        _sub(_sub(axis, "dynamics"), "damping", damping)


def _add_chassis(model, p):
    link = _sub(model, "link", name="base_link")
    center = (0, 0, p.chassis_z)
    _inertial(link, p.chassis_mass, box_inertia(p.chassis_mass, p.chassis_size), center)
    _shape(link, "chassis", _box(p.chassis_size), (*center, 0, 0, 0), CHASSIS_COLOR)
    _add_imu(link, p, center)


def _add_imu(link, p, xyz):
    sensor = _sub(link, "sensor", name="imu", type="imu")
    _sub(sensor, "pose", (*xyz, 0, 0, 0))
    _sub(sensor, "always_on", "true")
    _sub(sensor, "update_rate", p.imu_rate)
    _sub(sensor, "topic", IMU_TOPIC)
    # Same orientation as base_link, so only the linear acceleration differs
    # (by lever-arm terms) from a sensor at the base_link origin.
    _sub(sensor, "gz_frame_id", "base_link")
    imu = _sub(sensor, "imu")
    for group, stddev in (("angular_velocity", p.gyro_noise), ("linear_acceleration", p.accel_noise)):
        element = _sub(imu, group)
        for axis in "xyz":
            noise = _sub(_sub(element, axis), "noise", type="gaussian")
            _sub(noise, "mean", 0.0)
            _sub(noise, "stddev", stddev)


def _add_rocker(model, p, side, sign):
    name = f"rocker_{side}"
    link = _sub(model, "link", name=name)
    _sub(link, "pose", (0, sign * p.pivot_y, p.pivot_z, 0, 0, 0))
    inset = -sign * p.arm_inset  # toward the chassis
    moments = rocker_inertia(p.rocker_mass, p.wheel_dx, p.wheel_dz)
    _inertial(link, p.rocker_mass, moments, (0, inset, p.wheel_dz / 2))
    bar = (math.hypot(p.wheel_dx, p.wheel_dz), p.arm_thickness, p.arm_height)
    pitch = math.atan2(-p.wheel_dz, p.wheel_dx)  # turns the bar from +x down to the front hub
    for end, _, ahead in ENDS:
        pose = (ahead * p.wheel_dx / 2, inset, p.wheel_dz / 2, 0, ahead * pitch, 0)
        _shape(link, f"{end}_arm", _box(bar), pose, ROCKER_COLOR)
    _revolute(model, f"{name}_joint", "base_link", name, -p.rocker_limit, p.rocker_limit,
              damping=p.rocker_damping)


def _add_wheel(model, p, side, name, sign, ahead):
    link = _sub(model, "link", name=name)
    _sub(link, "pose", (ahead * p.wheel_dx, sign * p.pivot_y, p.pivot_z + p.wheel_dz, 0, 0, 0))
    _inertial(link, p.wheel_mass, wheel_inertia(p.wheel_mass, p.wheel_radius, p.wheel_width))
    # A cylinder runs along its z axis; roll it onto the axle.
    _shape(link, "tire", _cylinder(p.wheel_radius, p.wheel_width), (0, 0, 0, math.pi / 2, 0, 0),
           TIRE_COLOR, _tire_surface(p))
    _revolute(model, f"{name}_joint", f"rocker_{side}", name, -1e16, 1e16,
              effort=p.wheel_effort, velocity=p.wheel_speed)


def _add_plugins(model, p):
    diff = _sub(model, "plugin", filename="RockerDifferential", name="rover_sim::RockerDifferential")
    _sub(diff, "left_joint", "rocker_left_joint")
    _sub(diff, "right_joint", "rocker_right_joint")
    _sub(diff, "stiffness", p.diff_stiffness)
    _sub(diff, "damping", p.diff_damping)

    drive = _sub(model, "plugin", filename="gz-sim-diff-drive-system", name="gz::sim::systems::DiffDrive")
    for joint in ("wheel_fl_joint", "wheel_rl_joint"):
        _sub(drive, "left_joint", joint)
    for joint in ("wheel_fr_joint", "wheel_rr_joint"):
        _sub(drive, "right_joint", joint)
    _sub(drive, "wheel_separation", 2 * p.pivot_y)
    _sub(drive, "wheel_radius", p.wheel_radius)
    _sub(drive, "topic", CMD_VEL_TOPIC)
    _sub(drive, "odom_topic", ODOM_TOPIC)
    _sub(drive, "tf_topic", TF_TOPIC)
    _sub(drive, "frame_id", "odom")
    _sub(drive, "child_frame_id", "base_link")
    _sub(drive, "odom_publish_frequency", 50)

    states = _sub(model, "plugin", filename="gz-sim-joint-state-publisher-system",
                  name="gz::sim::systems::JointStatePublisher")
    _sub(states, "topic", JOINT_STATE_TOPIC)

    truth = _sub(model, "plugin", filename="gz-sim-odometry-publisher-system",
                 name="gz::sim::systems::OdometryPublisher")
    _sub(truth, "odom_topic", GROUND_TRUTH_TOPIC)
    _sub(truth, "odom_frame", "world")
    _sub(truth, "robot_base_frame", "base_link")
    _sub(truth, "dimensions", 3)
    _sub(truth, "odom_publish_frequency", 50)


def main():
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    out = MODEL_DIR / "model.sdf"
    out.write_text(build_sdf(Params()))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run to verify pass**

Run: `pixi run python -m unittest sim/tests/test_gen_model.py -v`
Expected: 12 tests OK.

- [ ] **Step 5: Model manifest** — `sim/models/rover/model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>rover</name>
  <version>0.1</version>
  <sdf version="1.11">model.sdf</sdf>
  <description>
    Generic 4-wheel rocker-differential skid-steer rover.
    model.sdf is generated by sim/gen_model.py; edit Params there.
  </description>
</model>
```

- [ ] **Step 6: Generate**

Run: `pixi run sim-model && pixi run gz sdf -k sim/models/rover/model.sdf`
Expected: `wrote …/sim/models/rover/model.sdf`, then `Valid.`

---

### Task 4: Differential plugin + physics tests (TDD)

**Files:**
- Create: `sim/tests/simulate.py`
- Create: `sim/tests/test_rover_sim.py`
- Create: `sim/CMakeLists.txt`
- Create: `sim/plugins/rocker_differential.cpp`

- [ ] **Step 1: Headless-sim helper** — `sim/tests/simulate.py`:

```python
"""Run the rover headless in Gazebo for the tests.

Gazebo's Python TestFixture runs the server inside this process, so a run is
repeatable: commands go out over gz-transport every 20 ms of sim time, and
the state is read from the entity-component manager after the last step.
"""
import os
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
import gen_model  # noqa: E402

# Set before Gazebo starts: where model://rover and the plugin live, and a
# private transport partition so tests never talk to a running simulation.
os.environ["GZ_SIM_RESOURCE_PATH"] = str(SIM_DIR / "models")
os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(SIM_DIR / "build")
os.environ["GZ_PARTITION"] = f"rover_sim_test_{os.getpid()}"
os.environ.setdefault("GZ_IP", "127.0.0.1")

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
    if world is None:
        with tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False) as f:
            f.write(world_sdf(extra, spawn_z))
        world_path = f.name
    else:
        world_path = str(world)
    node = Node()
    messages = {topic: [] for topic, _ in subscribe}
    for topic, msg_type in subscribe:
        node.subscribe(msg_type, topic, messages[topic].append)
    publisher = node.advertise(gen_model.CMD_VEL_TOPIC, Twist)
    twist = Twist()
    twist.linear.x, twist.angular.z = cmd
    handles = {}
    last = {}

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

    fixture = TestFixture(world_path)
    fixture.on_pre_update(pre_update)
    fixture.on_post_update(post_update)
    fixture.finalize()
    fixture.server().run(True, round(seconds * 1000), False)
    if world is None:
        os.unlink(world_path)
    rockers = {n: q[0] for n, q in last["rockers"].items()}
    return State(rockers, {n: _xyzrpy(pose) for n, pose in last["poses"].items()}, messages)


def _xyzrpy(pose):
    p, r = pose.pos(), pose.rot().euler()
    return (p.x(), p.y(), p.z(), r.x(), r.y(), r.z())
```

- [ ] **Step 2: Write the failing physics tests** — `sim/tests/test_rover_sim.py`:

```python
#!/usr/bin/env python3
"""Headless physics tests for the rover model (pixi run sim-test)."""
import math
import unittest

from simulate import SIM_DIR, gen_model, simulate

P = gen_model.Params()
STEP = 0.10
BLOCK_UNDER_FL = f"""
    <model name="block">
      <static>true</static>
      <pose>{P.wheel_dx} {P.pivot_y} {STEP / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.4 0.3 {STEP}</size></box></geometry></collision>
      </link>
    </model>"""


class Suspension(unittest.TestCase):
    def test_rests_level_on_flat_ground(self):
        s = simulate(2.0)
        _, _, z, roll, pitch, _ = s.poses["base_link"]
        self.assertAlmostEqual(z, 0.0, delta=0.02)
        self.assertLess(abs(roll), math.radians(1))
        self.assertLess(abs(pitch), math.radians(1))
        for joint, q in s.rockers.items():
            self.assertAlmostEqual(q, 0.0, delta=0.01, msg=joint)

    def test_differential_keeps_all_wheels_down(self):
        s = simulate(2.0, extra=BLOCK_UNDER_FL, spawn_z=STEP + 0.02)
        q_l, q_r = s.rockers["rocker_left_joint"], s.rockers["rocker_right_joint"]
        # The left rocker pitches front-up by atan(step / wheelbase); the
        # differential puts the body halfway, so each rocker sees half of it.
        self.assertLess(abs(q_l + q_r), 0.005)
        self.assertAlmostEqual(q_l, -math.atan(STEP / (2 * P.wheel_dx)) / 2, delta=0.01)
        # A rigid chassis would leave one wheel in the air.
        ground = {"wheel_fl": STEP, "wheel_fr": 0.0, "wheel_rl": 0.0, "wheel_rr": 0.0}
        for wheel, height in ground.items():
            self.assertAlmostEqual(s.poses[wheel][2], height + P.wheel_radius, delta=0.02, msg=wheel)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 3: Run to verify failure** (plugin not built yet)

Run: `pixi run python -m unittest discover -s sim/tests -p 'test_rover_sim.py' -v`
Expected: `test_differential_keeps_all_wheels_down` FAILS (rockers uncoupled: `|q_l + q_r|` large or a wheel off its surface); Gazebo logs that it failed to load `RockerDifferential`.

- [ ] **Step 4: CMake** — `sim/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.20)
project(rover_sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

# Models load it as <plugin filename="RockerDifferential">; Gazebo searches
# GZ_SIM_SYSTEM_PLUGIN_PATH, which sim/run.sh and the tests point here.
add_library(RockerDifferential SHARED plugins/rocker_differential.cpp)
target_link_libraries(RockerDifferential PRIVATE gz-sim8::gz-sim8 gz-plugin2::register)
target_compile_options(RockerDifferential PRIVATE -Wall -Wextra -Wpedantic)
```

- [ ] **Step 5: The plugin** — `sim/plugins/rocker_differential.cpp`:

```cpp
// RockerDifferential: a gz-sim system that couples two rocker joints the way
// a differential does.
//
// Every step it applies the same torque to both joints,
//   tau = -k (q_L + q_R) - c (qdot_L + qdot_R),
// a stiff spring-damper on the motion a differential forbids (both rockers
// turning the same way relative to the body). The allowed motion, opposite
// rotation, stays free. Equal torque on both joints is what a real
// differential does: it splits the reaction torque between the two sides.
//
// SDF: <left_joint>, <right_joint>, <stiffness> [N m/rad], <damping> [N m s/rad].
//
// Why not an SDF <mimic> joint: in Gazebo Harmonic only Bullet-Featherstone
// implements mimic, and Bullet cannot skid-steer this rover (sim/README.md).

#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

namespace rover_sim {

class RockerDifferential : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager&) override {
    const gz::sim::Model model(entity);
    const auto left_name = sdf->Get<std::string>("left_joint");
    const auto right_name = sdf->Get<std::string>("right_joint");
    left_ = gz::sim::Joint(model.JointByName(ecm, left_name));
    right_ = gz::sim::Joint(model.JointByName(ecm, right_name));
    if (!left_.Valid(ecm) || !right_.Valid(ecm)) {
      gzerr << "RockerDifferential: <left_joint> '" << left_name << "' and <right_joint> '"
            << right_name << "' must both be joints of model '" << model.Name(ecm)
            << "'; the differential is disabled.\n";
      return;
    }
    stiffness_ = sdf->Get<double>("stiffness", stiffness_).first;
    damping_ = sdf->Get<double>("damping", damping_).first;
    for (auto* joint : {&left_, &right_}) {
      joint->EnablePositionCheck(ecm, true);
      joint->EnableVelocityCheck(ecm, true);
    }
    enabled_ = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (!enabled_ || info.paused) {
      return;
    }
    const auto ql = left_.Position(ecm);
    const auto qr = right_.Position(ecm);
    const auto vl = left_.Velocity(ecm);
    const auto vr = right_.Velocity(ecm);
    // Empty until physics fills the components during the first step.
    if (!ql || !qr || !vl || !vr || ql->empty() || qr->empty() || vl->empty() || vr->empty()) {
      return;
    }
    const double tau = -stiffness_ * ((*ql)[0] + (*qr)[0]) - damping_ * ((*vl)[0] + (*vr)[0]);
    left_.SetForce(ecm, {tau});
    right_.SetForce(ecm, {tau});
  }

 private:
  gz::sim::Joint left_;
  gz::sim::Joint right_;
  double stiffness_ = 5000.0;
  double damping_ = 100.0;
  bool enabled_ = false;
};

}  // namespace rover_sim

GZ_ADD_PLUGIN(rover_sim::RockerDifferential, gz::sim::System,
              rover_sim::RockerDifferential::ISystemConfigure,
              rover_sim::RockerDifferential::ISystemPreUpdate)
```

- [ ] **Step 6: Build**

Run: `pixi run sim-build`
Expected: `Built target RockerDifferential`, no warnings from our file.

- [ ] **Step 7: Run to verify pass**

Run: `pixi run python -m unittest discover -s sim/tests -p 'test_rover_sim.py' -v`
Expected: 2 tests OK (prototype measured `q_l = −0.0557`, `q_r = +0.0558`).

---

### Task 5: Driving and topic tests

**Files:**
- Modify: `sim/tests/test_rover_sim.py`

- [ ] **Step 1: Add the tests** before the `if __name__` block:

```python
class Drive(unittest.TestCase):
    def test_drives_straight(self):
        s = simulate(4.0, cmd=(0.5, 0.0))
        x, y, _, _, _, yaw = s.poses["base_link"]
        self.assertTrue(1.7 <= x <= 2.1, f"x = {x}")
        self.assertLess(abs(y), 0.1)
        self.assertLess(abs(yaw), 0.05)

    def test_turns_in_place(self):
        s = simulate(4.0, cmd=(0.0, 0.5))
        x, y, _, _, _, yaw = s.poses["base_link"]
        # Commanded 2 rad; skid-steer scrub makes it turn slower (sim/README.md).
        self.assertTrue(1.2 <= yaw <= 2.05, f"yaw = {yaw}")
        self.assertLess(math.hypot(x, y), 0.15)


def frame_id(header):
    return next(d.value[0] for d in header.data if d.key == "frame_id")


class Topics(unittest.TestCase):
    def test_sensors_and_odometry_publish(self):
        from gz.msgs10.imu_pb2 import IMU
        from gz.msgs10.model_pb2 import Model
        from gz.msgs10.odometry_pb2 import Odometry

        topics = [(gen_model.IMU_TOPIC, IMU), (gen_model.JOINT_STATE_TOPIC, Model),
                  (gen_model.ODOM_TOPIC, Odometry), (gen_model.GROUND_TRUTH_TOPIC, Odometry)]
        s = simulate(1.0, subscribe=topics)
        for topic, _ in topics:
            self.assertTrue(s.messages[topic], f"nothing on {topic}")
        imu = s.messages[gen_model.IMU_TOPIC][-1]
        self.assertEqual(frame_id(imu.header), "base_link")
        a = imu.linear_acceleration
        self.assertAlmostEqual(math.sqrt(a.x**2 + a.y**2 + a.z**2), 9.8, delta=0.3)
        joints = {j.name for j in s.messages[gen_model.JOINT_STATE_TOPIC][-1].joint}
        self.assertEqual(joints, {"rocker_left_joint", "rocker_right_joint", "wheel_fl_joint",
                                  "wheel_rl_joint", "wheel_fr_joint", "wheel_rr_joint"})
```

- [ ] **Step 2: Run**

Run: `pixi run python -m unittest discover -s sim/tests -p 'test_rover_sim.py' -v`
Expected: 5 tests OK (prototype: straight 1.96 m, turn 1.74 rad). If `frame_id` is not `base_link`, `gz_frame_id` is not honored: check the gz-sensors version and fix the IMU element rather than loosening the test.

---

### Task 6: Demo world

**Files:**
- Create: `sim/worlds/rover_test.sdf`
- Modify: `sim/tests/test_rover_sim.py`

- [ ] **Step 1: Write the failing test** — add:

```python
class DemoWorld(unittest.TestCase):
    def test_loads_and_rover_settles(self):
        s = simulate(1.0, world=SIM_DIR / "worlds" / "rover_test.sdf")
        _, _, z, roll, pitch, _ = s.poses["base_link"]
        self.assertAlmostEqual(z, 0.0, delta=0.02)
        self.assertLess(max(abs(roll), abs(pitch)), math.radians(1))
```

- [ ] **Step 2: Run to verify failure**

Run: `pixi run python -m unittest discover -s sim/tests -p 'test_rover_sim.py' -k DemoWorld -v`
Expected: ERROR (world file missing).

- [ ] **Step 3: Write the world** — `sim/worlds/rover_test.sdf`. Ramp numbers: 15° slope, 0.3 m rise → slope length 0.3 / sin 15° = 1.159 m, run 1.1196 m; each 0.2 m thick ramp box is centered half its thickness below the middle of its top face.

```xml
<?xml version="1.0"?>
<!--
  Rover test ground. The rover starts at the origin facing +x:
    x 2-3 m   10 cm step under the left wheels only (works the differential)
    x 5-8.7 m 15 degree ramp up to a 0.3 m platform and back down
  plus a few rocks off to the sides. Run: pixi run sim
-->
<sdf version="1.11">
  <world name="rover_test">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.2 -0.9</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.55 0.45 0.35 1</ambient><diffuse>0.55 0.45 0.35 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="step_left">
      <static>true</static>
      <pose>2.5 0.4 0.05 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>1.0 0.4 0.1</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>1.0 0.4 0.1</size></box></geometry>
          <material><ambient>0.4 0.4 0.45 1</ambient><diffuse>0.4 0.4 0.45 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="ramp">
      <static>true</static>
      <link name="up">
        <pose>5.58569 0 0.053407 0 -0.261799 0</pose>
        <collision name="collision"><geometry><box><size>1.159 2.0 0.2</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>1.159 2.0 0.2</size></box></geometry>
          <material><ambient>0.45 0.42 0.4 1</ambient><diffuse>0.45 0.42 0.4 1</diffuse></material>
        </visual>
      </link>
      <link name="platform">
        <pose>6.869615 0 0.15 0 0 0</pose>
        <collision name="collision"><geometry><box><size>1.5 2.0 0.3</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>1.5 2.0 0.3</size></box></geometry>
          <material><ambient>0.45 0.42 0.4 1</ambient><diffuse>0.45 0.42 0.4 1</diffuse></material>
        </visual>
      </link>
      <link name="down">
        <pose>8.153541 0 0.053407 0 0.261799 0</pose>
        <collision name="collision"><geometry><box><size>1.159 2.0 0.2</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>1.159 2.0 0.2</size></box></geometry>
          <material><ambient>0.45 0.42 0.4 1</ambient><diffuse>0.45 0.42 0.4 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="rocks">
      <static>true</static>
      <link name="rock_1">
        <pose>1.5 -1.2 0.06 0 0 0.4</pose>
        <collision name="collision"><geometry><box><size>0.3 0.25 0.12</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>0.3 0.25 0.12</size></box></geometry>
          <material><ambient>0.35 0.3 0.28 1</ambient><diffuse>0.35 0.3 0.28 1</diffuse></material>
        </visual>
      </link>
      <link name="rock_2">
        <pose>3.5 1.5 0.04 0 0 0</pose>
        <collision name="collision"><geometry><sphere><radius>0.12</radius></sphere></geometry></collision>
        <visual name="visual">
          <geometry><sphere><radius>0.12</radius></sphere></geometry>
          <material><ambient>0.35 0.3 0.28 1</ambient><diffuse>0.35 0.3 0.28 1</diffuse></material>
        </visual>
      </link>
      <link name="rock_3">
        <pose>4.0 -0.8 0.075 0 0 -0.3</pose>
        <collision name="collision"><geometry><box><size>0.4 0.3 0.15</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>0.4 0.3 0.15</size></box></geometry>
          <material><ambient>0.35 0.3 0.28 1</ambient><diffuse>0.35 0.3 0.28 1</diffuse></material>
        </visual>
      </link>
    </model>

    <include>
      <uri>model://rover</uri>
      <pose>0 0 0.02 0 0 0</pose>
    </include>
  </world>
</sdf>
```

- [ ] **Step 4: Run to verify pass**

Run: `pixi run python -m unittest discover -s sim/tests -p 'test_rover_sim.py' -v`
Expected: 6 tests OK.

- [ ] **Step 5: Check the obstacles are drivable** (exploratory, not a unit test): a throwaway script calling `simulate(12.0, cmd=(0.4, 0.0), world=SIM_DIR / "worlds" / "rover_test.sdf")` should end with x > 4 (climbed the 10 cm step). If it stalls at the step, lower the step to 8 cm and update the spec and README.

---

### Task 7: ROS 2 bridge and launcher

**Files:**
- Create: `sim/bridge.yaml`
- Create: `sim/run.sh`

- [ ] **Step 1: Bridge config** — `sim/bridge.yaml`:

```yaml
# ros_gz_bridge config for the rover (pixi run sim-bridge).
# gz topics are set in sim/gen_model.py.
- ros_topic_name: /cmd_vel
  gz_topic_name: /model/rover/cmd_vel
  ros_type_name: geometry_msgs/msg/Twist
  gz_type_name: gz.msgs.Twist
  direction: ROS_TO_GZ
- ros_topic_name: /odom
  gz_topic_name: /model/rover/odometry
  ros_type_name: nav_msgs/msg/Odometry
  gz_type_name: gz.msgs.Odometry
  direction: GZ_TO_ROS
- ros_topic_name: /ground_truth/odom
  gz_topic_name: /model/rover/ground_truth
  ros_type_name: nav_msgs/msg/Odometry
  gz_type_name: gz.msgs.Odometry
  direction: GZ_TO_ROS
- ros_topic_name: /joint_states
  gz_topic_name: /model/rover/joint_states
  ros_type_name: sensor_msgs/msg/JointState
  gz_type_name: gz.msgs.Model
  direction: GZ_TO_ROS
- ros_topic_name: /imu
  gz_topic_name: /model/rover/imu
  ros_type_name: sensor_msgs/msg/Imu
  gz_type_name: gz.msgs.IMU
  direction: GZ_TO_ROS
- ros_topic_name: /tf
  gz_topic_name: /model/rover/tf
  ros_type_name: tf2_msgs/msg/TFMessage
  gz_type_name: gz.msgs.Pose_V
  direction: GZ_TO_ROS
- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS
```

- [ ] **Step 2: Launcher** — `sim/run.sh`, then `chmod +x sim/run.sh`:

```bash
#!/usr/bin/env bash
# Start the rover simulation: pixi run sim [world.sdf]
# macOS cannot run Gazebo's server and GUI in one process, so the server runs
# in the background and the GUI in the foreground; closing the GUI stops both.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
export GZ_SIM_RESOURCE_PATH="$here/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$here/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
world="${1:-$here/worlds/rover_test.sdf}"

if [[ "$(uname)" == "Darwin" ]]; then
  gz sim -s -r "$world" &
  server=$!
  trap 'kill "$server" 2>/dev/null || true' EXIT
  gz sim -g
else
  gz sim -r "$world"
fi
```

- [ ] **Step 3: Verify server + bridge end to end** (manual; the GUI needs a display):
  1. `GZ_SIM_RESOURCE_PATH=sim/models GZ_SIM_SYSTEM_PLUGIN_PATH=sim/build pixi run gz sim -s -r sim/worlds/rover_test.sdf &`
  2. `pixi run gz topic -l` lists `/model/rover/cmd_vel`, `/model/rover/odometry`, `/model/rover/joint_states`, `/model/rover/imu`, `/model/rover/ground_truth`, `/model/rover/tf`, `/clock`.
  3. `pixi run sim-bridge &`, then `pixi run ros2 topic echo --once /joint_states` prints six joints, and `pixi run ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}'` moves the rover (`/ground_truth/odom` x grows).
  4. Stop the background processes.

---

### Task 8: Documentation and full run

**Files:**
- Create: `sim/README.md`

- [ ] **Step 1: README** — `sim/README.md`, sections: Quick start (`pixi run sim`, `pixi run sim-bridge`, drive with `ros2 topic pub` / the GUI teleop, `pixi run sim-test`); Files; Topics (the bridge table); Parameters (point to `Params`, regenerate with `pixi run sim-model`); Design notes: **why the differential is a plugin and not a mimic joint** (the findings and numbers from the spec's "Why not mimic joints" section), how the plugin works and its compliance, tire friction anisotropy and the ~87 % turn rate; Known limitations; Troubleshooting (`GZ_IP=127.0.0.1` when gz-transport discovery fails; on macOS the server and GUI are separate processes).

- [ ] **Step 2: Full test run**

Run: `pixi run sim-test`
Expected: builds the plugin, regenerates the model, 18 tests OK (12 generator + 6 physics).

- [ ] **Step 3: Existing tests still pass**

Run: `pixi run driver-test`
Expected: all pass (nothing in `driver/` changed).
