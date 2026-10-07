#!/usr/bin/env python3
"""Generate the rover's Gazebo model, sim/models/rover/model.sdf, and with
it the driver station's cameras (viewers.py: the chase camera and the rover
eye).

Every dimension lives in Params (ChaseParams, EyeParams in viewers.py for the
cameras). The defaults are the placeholder geometry from
driver/include/rover_driver/config.hpp until the mechanical team has real
numbers: edit Params, then run `pixi run sim-model`.

Frames follow the driver: x forward, y left, z up, origin on the ground
midway between the four wheels with the rockers at zero.
"""
import math
from dataclasses import dataclass
from pathlib import Path

from urc import sdf  # the SDF writer every generated model shares

MODELS_DIR = Path(__file__).resolve().parent / "models"
MODEL_DIR = MODELS_DIR / "rover"

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
# The driver station's cameras live in viewers.py, which builds on Params;
# the station and the tests find their names here too (__getattr__).
VIEWER_NAMES = ("CHASE_MODEL", "CHASE_IMAGE_TOPIC", "CHASE_CMD_TOPIC", "CHASE_MODE_TOPIC", "CHASE_STATE_TOPIC",
                "EYE_MODEL", "EYE_IMAGE_TOPIC", "EYE_LOOK_TOPIC", "EYE_STATE_TOPIC",
                "ChaseParams", "EyeParams", "build_chase_sdf", "build_eye_sdf")


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
    root, model = sdf.model_root("rover")
    _add_chassis(model, p)
    _add_camera_head(model, p)
    for side, s, sign in SIDES:
        _add_rocker(model, p, side, sign)
        for _, e, ahead in ENDS:
            _add_wheel(model, p, side, f"wheel_{e}{s}", sign, ahead)
    _add_plugins(model, p)
    return sdf.document(root)


def _visual(link, name, geometry, pose, color, emissive=None):
    """A visual without a collision (sensor housings, the LED: no mass, no contact)."""
    return sdf.visual(link, name, geometry, pose, color, emissive=emissive, plain=True)


def _shape(link, name, geometry, pose, color):
    """A collision and a matching visual."""
    sdf.collision(link, name, geometry, pose)
    _visual(link, name, geometry, pose, color)


def _add_chassis(model, p):
    center = (0, 0, p.chassis_z)
    link = sdf.link(model, "base_link", None, p.chassis_mass, sdf.box_inertia(p.chassis_mass, p.chassis_size), center)
    _shape(link, "chassis", sdf.box(p.chassis_size), (*center, 0, 0, 0), CHASSIS_COLOR)
    _add_imu(link, p, center)
    _add_gnss(link, p)
    _mast(link, p, "camera_mast", p.camera_xyz, p.camera_xyz[2] - p.camera_pan_height - 0.01)
    _add_led(link, p)


def _add_imu(link, p, xyz):
    sensor = sdf.sub(link, "sensor", name="imu", type="imu")
    sdf.pose(sensor, xyz)
    sdf.sub(sensor, "always_on", True)
    sdf.sub(sensor, "update_rate", p.imu_rate)
    sdf.sub(sensor, "topic", IMU_TOPIC)
    # Same orientation as base_link, so only the linear acceleration differs
    # (by lever-arm terms) from a sensor at the base_link origin.
    sdf.sub(sensor, "gz_frame_id", "base_link")
    imu = sdf.sub(sensor, "imu")
    for group, stddev in (("angular_velocity", p.gyro_noise), ("linear_acceleration", p.accel_noise)):
        element = sdf.sub(imu, group)
        for axis in "xyz":
            _noise(sdf.sub(element, axis), stddev)


def _noise(parent, stddev):
    noise = sdf.sub(parent, "noise", type="gaussian")
    sdf.sub(noise, "mean", 0.0)
    sdf.sub(noise, "stddev", stddev)


def _mast(link, p, name, xyz, top_z):
    """A thin pole from the chassis top up to top_z."""
    x, y, _ = xyz
    height = top_z - (p.chassis_z + p.chassis_size[2] / 2)
    _visual(link, name, sdf.cylinder(0.015, height), (x, y, top_z - height / 2, 0, 0, 0), ROCKER_COLOR)


def _add_gnss(link, p):
    _mast(link, p, "gnss_mast", p.gnss_xyz, p.gnss_xyz[2])
    _visual(link, "gnss_antenna", sdf.cylinder(0.06, 0.02), (*p.gnss_xyz, 0, 0, 0), (0.9, 0.9, 0.9, 1))
    sensor = sdf.sub(link, "sensor", name="gnss", type="navsat")
    sdf.pose(sensor, p.gnss_xyz)
    sdf.sub(sensor, "always_on", True)
    sdf.sub(sensor, "update_rate", p.gnss_rate)
    sdf.sub(sensor, "topic", NAVSAT_TOPIC)
    sdf.sub(sensor, "gz_frame_id", "gnss")
    sensing = sdf.sub(sdf.sub(sensor, "navsat"), "position_sensing")
    for axis, stddev in (("horizontal", p.gnss_noise_horizontal / METERS_PER_DEGREE),
                         ("vertical", p.gnss_noise_vertical)):
        _noise(sdf.sub(sensing, axis), stddev)


def _add_camera_head(model, p):
    """The RGB-D camera on a pan-tilt head. Each joint has a
    JointPositionController in velocity mode: it slews at camera_head_speed to
    the angle last sent on HEAD_TOPIC and holds it (the astronaut's arms in
    urc/props.py work the same way)."""
    x, y, z = p.camera_xyz
    pan = sdf.link(model, "camera_pan_link", (x, y, z - p.camera_pan_height), p.camera_pan_mass,
                   sdf.box_inertia(p.camera_pan_mass, (0.06, 0.15, 0.06)))
    _visual(pan, "turntable", sdf.cylinder(0.03, 0.02), (0, 0, 0, 0, 0, 0), ROCKER_COLOR)
    for side, sign in (("left", 1), ("right", -1)):
        _visual(pan, f"yoke_{side}", sdf.box((0.03, 0.008, p.camera_pan_height + 0.02)),
                (0, sign * 0.07, p.camera_pan_height / 2, 0, 0, 0), ROCKER_COLOR)
    _visual(pan, "yoke_base", sdf.box((0.03, 0.148, 0.008)), (0, 0, 0.01, 0, 0, 0), ROCKER_COLOR)

    tilt = sdf.link(model, "camera_tilt_link", p.camera_xyz, p.camera_tilt_mass,
                    sdf.box_inertia(p.camera_tilt_mass, (0.04, 0.12, 0.04)))
    _visual(tilt, "camera", sdf.box((0.04, 0.12, 0.04)), (0, 0, 0, 0, 0, 0), (0.1, 0.1, 0.1, 1))
    _visual(tilt, "lens", sdf.cylinder(0.014, 0.012), (0.024, 0.03, 0, 0, math.pi / 2, 0), (0.05, 0.08, 0.12, 1))
    sensor = sdf.sub(tilt, "sensor", name="camera", type="rgbd_camera")
    sdf.pose(sensor, (0, 0, 0))
    sdf.sub(sensor, "always_on", True)
    sdf.sub(sensor, "update_rate", p.camera_rate)
    sdf.sub(sensor, "topic", CAMERA_TOPIC)
    sdf.sub(sensor, "gz_frame_id", "camera")
    sdf.camera(sensor, p.camera_hfov, p.camera_size, p.camera_clip)

    head = ((PAN_JOINT, "base_link", "camera_pan_link", (0, 0, 1), (-p.camera_pan_limit, p.camera_pan_limit), 0.0),
            (TILT_JOINT, "camera_pan_link", "camera_tilt_link", (0, 1, 0), p.camera_tilt_limits, p.camera_pitch))
    for name, parent, child, xyz, (lower, upper), start in head:
        sdf.joint(model, name, "revolute", parent, child, xyz, lower, upper, effort=p.camera_head_effort,
                  velocity=p.camera_head_speed, damping=0.01)
        sdf.plugin(model, "gz-sim-joint-position-controller-system", "gz::sim::systems::JointPositionController",
                   joint_name=name, topic=HEAD_TOPIC.format(joint=name), use_velocity_commands=True, p_gain=10.0,
                   cmd_max=p.camera_head_speed, cmd_min=-p.camera_head_speed, initial_position=start)


def _add_led(link, p):
    # Off (dark grey) until the referee shows the colour on LED_TOPIC.
    _visual(link, LED_VISUAL.removesuffix("_visual"), sdf.box((0.02, 0.16, 0.06)), (*p.led_xyz, 0, 0, 0),
            (0.15, 0.15, 0.15, 1), emissive=(0, 0, 0, 1))


def _add_rocker(model, p, side, sign):
    name = f"rocker_{side}"
    inset = -sign * p.arm_inset  # toward the chassis
    link = sdf.link(model, name, (0, sign * p.pivot_y, p.pivot_z, 0, 0, 0), p.rocker_mass,
                    rocker_inertia(p.rocker_mass, p.wheel_dx, p.wheel_dz), (0, inset, p.wheel_dz / 2))
    bar = (math.hypot(p.wheel_dx, p.wheel_dz), p.arm_thickness, p.arm_height)
    pitch = math.atan2(-p.wheel_dz, p.wheel_dx)  # turns the bar from +x down to the front hub
    for end, _, ahead in ENDS:
        pose = (ahead * p.wheel_dx / 2, inset, p.wheel_dz / 2, 0, ahead * pitch, 0)
        _shape(link, f"{end}_arm", sdf.box(bar), pose, ROCKER_COLOR)
    sdf.joint(model, f"{name}_joint", "revolute", "base_link", name, (0, 1, 0), -p.rocker_limit, p.rocker_limit,
              damping=p.rocker_damping)


def _add_wheel(model, p, side, name, sign, ahead):
    link = sdf.link(model, name, (ahead * p.wheel_dx, sign * p.pivot_y, p.pivot_z + p.wheel_dz, 0, 0, 0),
                    p.wheel_mass, wheel_inertia(p.wheel_mass, p.wheel_radius, p.wheel_width))
    # A cylinder runs along its z axis; roll it onto the axle. fdir1 is the
    # axle in the collision frame: mu acts along it (lateral), mu2 across it
    # (longitudinal).
    tire, pose = sdf.cylinder(p.wheel_radius, p.wheel_width), (0, 0, 0, math.pi / 2, 0, 0)
    sdf.collision(link, "tire", tire, pose, mu=p.mu_lateral, mu2=p.mu_longitudinal, fdir1=(0, 0, 1))
    _visual(link, "tire", tire, pose, TIRE_COLOR)
    sdf.joint(model, f"{name}_joint", "revolute", f"rocker_{side}", name, (0, 1, 0), -1e16, 1e16,
              effort=p.wheel_effort, velocity=p.wheel_speed)


def _add_plugins(model, p):
    sdf.plugin(model, "RockerDifferential", "rover_sim::RockerDifferential", left_joint="rocker_left_joint",
               right_joint="rocker_right_joint", stiffness=p.diff_stiffness, damping=p.diff_damping)
    sdf.plugin(model, "gz-sim-diff-drive-system", "gz::sim::systems::DiffDrive",
               left_joint=["wheel_fl_joint", "wheel_rl_joint"], right_joint=["wheel_fr_joint", "wheel_rr_joint"],
               wheel_separation=2 * p.pivot_y, wheel_radius=p.wheel_radius, topic=CMD_VEL_TOPIC,
               odom_topic=ODOM_TOPIC, tf_topic=TF_TOPIC, frame_id="odom", child_frame_id="base_link",
               odom_publish_frequency=50)
    sdf.plugin(model, "gz-sim-joint-state-publisher-system", "gz::sim::systems::JointStatePublisher",
               topic=JOINT_STATE_TOPIC)
    sdf.plugin(model, "gz-sim-odometry-publisher-system", "gz::sim::systems::OdometryPublisher",
               odom_topic=GROUND_TRUTH_TOPIC, odom_frame="world", robot_base_frame="base_link", dimensions=3,
               odom_publish_frequency=50)


def __getattr__(name):
    """The viewer names (VIEWER_NAMES) from viewers.py, imported on first use:
    viewers imports this module for Params."""
    if name in VIEWER_NAMES:
        import viewers
        return getattr(viewers, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def main():
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    out = MODEL_DIR / "model.sdf"
    out.write_text(build_sdf(Params()))
    print(f"wrote {out}")
    import viewers  # here, not at the top: viewers imports this module
    viewers.write_all(MODELS_DIR)


if __name__ == "__main__":
    main()
