#!/usr/bin/env python3
"""Generate the rover's Gazebo model, sim/models/rover/model.sdf, and the
driver station's two cameras: the chase camera that follows it,
sim/models/chase_camera, and the rover eye at its camera pivot,
sim/models/eye_camera.

Every dimension lives in Params (ChaseParams, EyeParams for the cameras). The
defaults are the placeholder geometry from driver/include/rover_driver/config.hpp
until the mechanical team has real numbers: edit Params, then run
`pixi run sim-model`.

Frames follow the driver: x forward, y left, z up, origin on the ground
midway between the four wheels with the rockers at zero.
"""
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

MODELS_DIR = Path(__file__).resolve().parent / "models"
MODEL_DIR = MODELS_DIR / "rover"
CHASE_MODEL_DIR = MODELS_DIR / "chase_camera"
EYE_MODEL_DIR = MODELS_DIR / "eye_camera"

CMD_VEL_TOPIC = "/model/rover/cmd_vel"
ODOM_TOPIC = "/model/rover/odometry"
TF_TOPIC = "/model/rover/tf"
JOINT_STATE_TOPIC = "/model/rover/joint_states"
GROUND_TRUTH_TOPIC = "/model/rover/ground_truth"
IMU_TOPIC = "/model/rover/imu"
NAVSAT_TOPIC = "/model/rover/navsat"
CAMERA_TOPIC = "/model/rover/camera"  # rgbd: /image, /depth_image, /points, /camera_info
LED_TOPIC = "/model/rover/led"  # set by the rover's software, shown by sim/referee.py
LED_VISUAL = "led_visual"
# The camera's pan-tilt head: gz.msgs.Double target angles [rad].
PAN_JOINT = "camera_pan_joint"
TILT_JOINT = "camera_tilt_joint"
HEAD_TOPIC = "/model/rover/joint/{joint}/0/cmd_pos"
METERS_PER_DEGREE = 111_320.0  # of latitude, for NavSat noise

# Chase camera (plugins/chase_camera.cpp has the details).
CHASE_MODEL = "chase_camera"
CHASE_IMAGE_TOPIC = "/chase_camera/image"
CHASE_CMD_TOPIC = "/chase_camera/cmd"  # gz.msgs.Vector3d: d yaw, d pitch [rad], zoom (distance *= e^z)
CHASE_MODE_TOPIC = "/chase_camera/mode"  # gz.msgs.StringMsg: follow, orbit, reset
CHASE_STATE_TOPIC = "/chase_camera/state"  # gz.msgs.StringMsg, JSON
# Rover eye: the same plugin in eye mode.
EYE_MODEL = "eye_camera"
EYE_IMAGE_TOPIC = "/eye_camera/image"
EYE_LOOK_TOPIC = "/eye_camera/look"  # gz.msgs.Vector3d: yaw (= pan), pitch (= tilt) [rad], absolute
EYE_STATE_TOPIC = "/eye_camera/state"  # gz.msgs.StringMsg, JSON


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
    # GNSS antenna (navsat sensor) on a short mast. Gazebo applies horizontal
    # noise to latitude and longitude in degrees; the generator converts at
    # 111.32 km per degree, so east-west noise is this times cos(latitude).
    gnss_xyz: tuple[float, float, float] = (-0.25, 0.0, 0.85)
    gnss_rate: float = 10.0  # [Hz]
    gnss_noise_horizontal: float = 0.5  # [m] stddev
    gnss_noise_vertical: float = 1.0  # [m] stddev
    # Front RGB-D camera on a pan-tilt head on a mast. camera_xyz is the
    # camera and the tilt axis; the pan axis is vertical through it.
    camera_xyz: tuple[float, float, float] = (0.35, 0.0, 0.85)
    camera_pitch: float = 0.12  # [rad] down: where the tilt starts and C centres it
    camera_rate: float = 15.0  # [Hz]
    camera_size: tuple[int, int] = (640, 480)
    camera_hfov: float = 1.5  # [rad]
    camera_clip: tuple[float, float] = (0.1, 40.0)  # [m] depth range
    # Pan-tilt head. Angles are from straight ahead and level; tilt is positive
    # down (rotation about +y), pan positive to the left (about +z).
    camera_pan_limit: float = 2.8  # [rad] each way
    camera_tilt_limits: tuple[float, float] = (-0.6, 1.2)  # [rad] (up, down)
    camera_head_speed: float = 2.0  # [rad/s]
    camera_head_effort: float = 5.0  # [N m]
    # Token masses: the camera's real mass is part of chassis_mass. Skid-steer
    # turning is sensitive to the centre of mass: 0.4 kg on the head moves it
    # 3 mm forward and made a 4 s turn in place wander 0.24 m instead of 0.
    camera_pan_mass: float = 0.02
    camera_tilt_mass: float = 0.02
    camera_pan_height: float = 0.05  # [m] pan bearing below the tilt axis
    # Status LED on the back of the rover (URC 2027 rule 1.e.ii).
    led_xyz: tuple[float, float, float] = (-0.41, 0.0, 0.47)


@dataclass(frozen=True)
class ChaseParams:
    """The driver station's third-person camera (plugins/chase_camera.cpp):
    a massless-looking body without gravity or collisions that the plugin
    places on a sphere around a point above the target every step."""
    target: str = "rover"
    look_height: float = 0.5  # [m] look-at point above the target's origin
    # Start offset: azimuth from behind the target, elevation above the horizon, range.
    yaw: float = 0.0  # [rad]
    pitch: float = 0.3  # [rad]
    distance: float = 5.0  # [m]
    # Pitch stays above the horizon so the camera never dips below the target.
    pitch_limits: tuple[float, float] = (0.05, 1.45)  # [rad]
    distance_limits: tuple[float, float] = (1.5, 80.0)  # [m]
    time_constant: float = 0.25  # [s] first-order smoothing of the view
    rate: float = 20.0  # [Hz]
    size: tuple[int, int] = (960, 540)
    hfov: float = 1.2  # [rad]
    clip: tuple[float, float] = (0.1, 2000.0)  # [m]


@dataclass(frozen=True)
class EyeParams:
    """The driver station's first-person camera (plugins/chase_camera.cpp in
    eye mode): it rides at the rover's camera pivot, Params.camera_xyz, where
    the pan and tilt axes meet, and turns there like the pan-tilt head and
    within its joint limits. Only the picture differs from the RGB-D sensor's."""
    target: str = "rover"
    rate: float = 20.0  # [Hz]
    size: tuple[int, int] = (960, 540)
    hfov: float = Params.camera_hfov  # [rad] as wide as the sensor
    # The near clip also hides the camera's own housing and yoke, all within
    # 0.1 m of the pivot; far reaches the horizon (the sensor stops at 40 m).
    clip: tuple[float, float] = (Params.camera_clip[0], 2000.0)  # [m]


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
    _add_camera_head(model, p)
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


def _revolute(model, name, parent, child, lower, upper, effort=None, velocity=None, damping=None, xyz=(0, 1, 0)):
    joint = _sub(model, "joint", name=name, type="revolute")
    _sub(joint, "parent", parent)
    _sub(joint, "child", child)
    axis = _sub(joint, "axis")
    _sub(axis, "xyz", xyz)
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
    _add_gnss(link, p)
    _mast(link, p, "camera_mast_visual", p.camera_xyz, p.camera_xyz[2] - p.camera_pan_height - 0.01)
    _add_led(link, p)


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


def _visual(link, name, geometry, pose, color, emissive=None):
    """A visual without a collision (sensor housings, the LED: no mass, no contact)."""
    element = _sub(link, "visual", name=name)
    _sub(element, "pose", pose)
    geometry(_sub(element, "geometry"))
    material = _sub(element, "material")
    _sub(material, "ambient", color)
    _sub(material, "diffuse", color)
    if emissive is not None:
        _sub(material, "emissive", emissive)
    return element


def _mast(link, p, name, xyz, top_z):
    """A thin pole from the chassis top up to top_z."""
    x, y, _ = xyz
    height = top_z - (p.chassis_z + p.chassis_size[2] / 2)
    _visual(link, name, _cylinder(0.015, height), (x, y, top_z - height / 2, 0, 0, 0), ROCKER_COLOR)


def _add_gnss(link, p):
    _mast(link, p, "gnss_mast_visual", p.gnss_xyz, p.gnss_xyz[2])
    _visual(link, "gnss_antenna_visual", _cylinder(0.06, 0.02), (*p.gnss_xyz, 0, 0, 0), (0.9, 0.9, 0.9, 1))
    sensor = _sub(link, "sensor", name="gnss", type="navsat")
    _sub(sensor, "pose", (*p.gnss_xyz, 0, 0, 0))
    _sub(sensor, "always_on", "true")
    _sub(sensor, "update_rate", p.gnss_rate)
    _sub(sensor, "topic", NAVSAT_TOPIC)
    _sub(sensor, "gz_frame_id", "gnss")
    sensing = _sub(_sub(sensor, "navsat"), "position_sensing")
    for axis, stddev in (("horizontal", p.gnss_noise_horizontal / METERS_PER_DEGREE),
                         ("vertical", p.gnss_noise_vertical)):
        noise = _sub(_sub(sensing, axis), "noise", type="gaussian")
        _sub(noise, "mean", 0.0)
        _sub(noise, "stddev", stddev)


def _add_camera_head(model, p):
    """The RGB-D camera on a pan-tilt head. Each joint has a
    JointPositionController in velocity mode: it slews at camera_head_speed to
    the angle last sent on HEAD_TOPIC and holds it (the astronaut's arms in
    urc/props.py work the same way)."""
    x, y, z = p.camera_xyz
    pan_xyz = (x, y, z - p.camera_pan_height)
    pan = _sub(model, "link", name="camera_pan_link")
    _sub(pan, "pose", (*pan_xyz, 0, 0, 0))
    _inertial(pan, p.camera_pan_mass, box_inertia(p.camera_pan_mass, (0.06, 0.15, 0.06)))
    _visual(pan, "turntable_visual", _cylinder(0.03, 0.02), (0, 0, 0, 0, 0, 0), ROCKER_COLOR)
    for side, sign in (("left", 1), ("right", -1)):
        _visual(pan, f"yoke_{side}_visual", _box((0.03, 0.008, p.camera_pan_height + 0.02)),
                (0, sign * 0.07, p.camera_pan_height / 2, 0, 0, 0), ROCKER_COLOR)
    _visual(pan, "yoke_base_visual", _box((0.03, 0.148, 0.008)), (0, 0, 0.01, 0, 0, 0), ROCKER_COLOR)

    tilt = _sub(model, "link", name="camera_tilt_link")
    _sub(tilt, "pose", (*p.camera_xyz, 0, 0, 0))
    _inertial(tilt, p.camera_tilt_mass, box_inertia(p.camera_tilt_mass, (0.04, 0.12, 0.04)))
    _visual(tilt, "camera_visual", _box((0.04, 0.12, 0.04)), (0, 0, 0, 0, 0, 0), (0.1, 0.1, 0.1, 1))
    _visual(tilt, "lens_visual", _cylinder(0.014, 0.012), (0.024, 0.03, 0, 0, math.pi / 2, 0), (0.05, 0.08, 0.12, 1))
    sensor = _sub(tilt, "sensor", name="camera", type="rgbd_camera")
    _sub(sensor, "pose", (0, 0, 0, 0, 0, 0))
    _sub(sensor, "always_on", "true")
    _sub(sensor, "update_rate", p.camera_rate)
    _sub(sensor, "topic", CAMERA_TOPIC)
    _sub(sensor, "gz_frame_id", "camera")
    camera = _sub(sensor, "camera")
    _sub(camera, "horizontal_fov", p.camera_hfov)
    image = _sub(camera, "image")
    _sub(image, "width", str(p.camera_size[0]))
    _sub(image, "height", str(p.camera_size[1]))
    clip = _sub(camera, "clip")
    _sub(clip, "near", p.camera_clip[0])
    _sub(clip, "far", p.camera_clip[1])

    head = ((PAN_JOINT, "base_link", "camera_pan_link", (0, 0, 1), (-p.camera_pan_limit, p.camera_pan_limit), 0.0),
            (TILT_JOINT, "camera_pan_link", "camera_tilt_link", (0, 1, 0), p.camera_tilt_limits, p.camera_pitch))
    for name, parent, child, xyz, (lower, upper), start in head:
        _revolute(model, name, parent, child, lower, upper, effort=p.camera_head_effort,
                  velocity=p.camera_head_speed, damping=0.01, xyz=xyz)
        controller = _sub(model, "plugin", filename="gz-sim-joint-position-controller-system",
                          name="gz::sim::systems::JointPositionController")
        _sub(controller, "joint_name", name)
        _sub(controller, "topic", HEAD_TOPIC.format(joint=name))
        _sub(controller, "use_velocity_commands", "true")
        _sub(controller, "p_gain", 10.0)
        _sub(controller, "cmd_max", p.camera_head_speed)
        _sub(controller, "cmd_min", -p.camera_head_speed)
        _sub(controller, "initial_position", start)


def _add_led(link, p):
    # Off (dark grey) until the referee shows the colour on LED_TOPIC.
    _visual(link, LED_VISUAL, _box((0.02, 0.16, 0.06)), (*p.led_xyz, 0, 0, 0), (0.15, 0.15, 0.15, 1),
            emissive=(0, 0, 0, 1))


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


def _viewer_sdf(name, topic, rate, size, hfov, clip, plugin):
    """A driver station camera as an SDF 1.11 document: one link without
    gravity or collisions (nothing changes its velocity, which stays zero, so
    only the ChaseCamera plugin's teleports move it) carrying an RGB camera,
    and the plugin with the (tag, value) elements in `plugin`."""
    sdf = ET.Element("sdf", version="1.11")
    model = _sub(sdf, "model", name=name)
    link = _sub(model, "link", name="link")
    _sub(link, "gravity", "false")
    _inertial(link, 0.1, box_inertia(0.1, (0.1, 0.1, 0.1)))
    sensor = _sub(link, "sensor", name="camera", type="camera")
    _sub(sensor, "always_on", "true")
    _sub(sensor, "update_rate", rate)
    _sub(sensor, "topic", topic)
    camera = _sub(sensor, "camera")
    _sub(camera, "horizontal_fov", hfov)
    image = _sub(camera, "image")
    _sub(image, "width", str(size[0]))
    _sub(image, "height", str(size[1]))
    _sub(image, "format", "R8G8B8")
    clip_element = _sub(camera, "clip")
    _sub(clip_element, "near", clip[0])
    _sub(clip_element, "far", clip[1])
    element = _sub(model, "plugin", filename="ChaseCamera", name="rover_sim::ChaseCamera")
    for tag, value in plugin:
        _sub(element, tag, value)
    ET.indent(sdf)
    return '<?xml version="1.0"?>\n' + ET.tostring(sdf, encoding="unicode") + "\n"


def build_chase_sdf(c: ChaseParams) -> str:
    """The chase camera as an SDF 1.11 document."""
    return _viewer_sdf(CHASE_MODEL, CHASE_IMAGE_TOPIC, c.rate, c.size, c.hfov, c.clip, (
        ("target", c.target), ("look_height", c.look_height),
        ("yaw", c.yaw), ("pitch", c.pitch), ("distance", c.distance),
        ("min_pitch", c.pitch_limits[0]), ("max_pitch", c.pitch_limits[1]),
        ("min_distance", c.distance_limits[0]), ("max_distance", c.distance_limits[1]),
        ("time_constant", c.time_constant),
        ("cmd_topic", CHASE_CMD_TOPIC), ("mode_topic", CHASE_MODE_TOPIC), ("state_topic", CHASE_STATE_TOPIC)))


def build_eye_sdf(e: EyeParams, p: Params) -> str:
    """The rover eye as an SDF 1.11 document: the plugin's eye mode at the
    camera pivot, starting and limited like the pan-tilt head."""
    return _viewer_sdf(EYE_MODEL, EYE_IMAGE_TOPIC, e.rate, e.size, e.hfov, e.clip, (
        ("target", e.target), ("mount", p.camera_xyz),
        ("yaw", 0.0), ("pitch", p.camera_pitch),
        ("min_yaw", -p.camera_pan_limit), ("max_yaw", p.camera_pan_limit),
        ("min_pitch", p.camera_tilt_limits[0]), ("max_pitch", p.camera_tilt_limits[1]),
        ("look_topic", EYE_LOOK_TOPIC), ("state_topic", EYE_STATE_TOPIC)))


def _model_config(name, description):
    config = ET.Element("model")
    _sub(config, "name", name)
    _sub(config, "version", "0.1")
    _sub(config, "sdf", "model.sdf", version="1.11")
    _sub(config, "description", description)
    ET.indent(config)
    return '<?xml version="1.0"?>\n' + ET.tostring(config, encoding="unicode") + "\n"


def main():
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    out = MODEL_DIR / "model.sdf"
    out.write_text(build_sdf(Params()))
    print(f"wrote {out}")
    cameras = ((CHASE_MODEL_DIR, CHASE_MODEL, build_chase_sdf(ChaseParams()),
                "Third-person camera for the driver station (sim/station), moved by the ChaseCamera plugin. "
                "Generated by sim/gen_model.py; edit ChaseParams there."),
               (EYE_MODEL_DIR, EYE_MODEL, build_eye_sdf(EyeParams(), Params()),
                "First-person camera for the driver station (sim/station): the ChaseCamera plugin keeps it at "
                "the rover's camera pivot. Generated by sim/gen_model.py; edit EyeParams there."))
    for directory, name, sdf, description in cameras:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "model.sdf").write_text(sdf)
        (directory / "model.config").write_text(_model_config(name, description))
        print(f"wrote {directory / 'model.sdf'}")


if __name__ == "__main__":
    main()
