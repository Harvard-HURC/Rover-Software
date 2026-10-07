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

DriveParams.mode picks the drivetrain (design spec
docs/superpowers/specs/2026-10-06-urc-realism-design.md, section 6, D22):
"physical", the default, plugins/rover_drivetrain.cpp (a DC motor per wheel
driving it by torque, every wheel contact gripping like the ground under
it), or "diffdrive", Gazebo's DiffDrive (every wheel a velocity servo on
anisotropic tyres), kept for A/B tests and cost comparisons. The camera and
the dust emitters are the same in both.
"""
import dataclasses
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
DRIVETRAIN_TOPIC = "/model/rover/drivetrain"  # physical drivetrain: gz.msgs.StringMsg, JSON (design spec 9.3)
DUST_TOPIC = "/model/rover/link/{link}/particle_emitter/{emitter}/cmd"  # gz.msgs.ParticleEmitter
DUST_SPRITE = "materials/textures/dust_puff.png"  # in the rover model: a soft dust puff (textures.dust_puff)
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
class DriveParams:
    """The drivetrain (design spec section 6). mode "physical", the default:
    plugins/rover_drivetrain.cpp, a DC motor per wheel driving it by torque,
    every wheel contact gripping like the ground under it; "diffdrive":
    Gazebo's DiffDrive, every wheel a velocity servo, kept for A/B tests and
    cost comparisons (D22). Motor numbers are typical placeholders until the
    drivetrain is chosen (D20, Q7: the prototype's validated set)."""
    mode: str = "physical"  # or "diffdrive"
    # A command older than cmd_timeout counts as zero, so a commander that died cannot leave the rover
    # driving (user decision 2026-10-06); 0 holds the last command, as Gazebo's GUI Teleop needs. The clock:
    # "wall" (a dead process) or "sim" (deterministic tests).
    cmd_timeout: float = 0.5  # [s]
    cmd_timeout_clock: str = "wall"
    track_multiplier: float = 1.0  # effective-track compensation; 1 = the raw skid-steer response (D24, Q10)
    accel: float = 8.0  # [rad/s^2] wheel setpoint ramp, 1.2 m/s^2 (A: until the driver team reports theirs, Q7)
    voltage: float = 24.0  # [V] (R: Husky A200 motor [18])
    resistance: float = 0.46  # [ohm] (R [18])
    kt: float = 0.0445  # [N m/A] (R [18])
    ke: float = 0.0445  # [V s/rad] (R [18])
    gear: float = 50.0  # (M: prototype; datasheet alternative 51, R [22])
    efficiency: float = 0.8  # (M: prototype; datasheet alternative 0.7)
    rotor_inertia: float = 1.2e-5  # [kg m^2] at the motor, 0.03 at the wheel (A)
    free_current: float = 1.0  # [A] no-load current (A)
    output_friction: float = 0.05  # [N m s/rad] (A)
    current_limit: float = 20.0  # [A] 35.6 N m at the wheel (M: prototype)
    driveline_stiffness: float = 1500.0  # [N m/rad] (A: 12 mm x 0.1 m steel shaft)
    driveline_damping: float = 2.0  # [N m s/rad] (A)
    backlash: float = 0.026  # [rad] 1.5 deg (A: IMS 0.8-2.5 deg [22])
    kp: float = 4.0  # [V/(rad/s)] (M: prototype tuning)
    ki: float = 40.0  # [V/rad] (M: prototype tuning)
    speed_filter: float = 0.005  # [s] measured-speed time constant, also the contact rule's (A)
    substeps: int = 4  # motor integration steps per 1 ms physics step
    # Wheel contacts (design spec 6.4): friction circle while sliding, an aligned box while sticking.
    v_stribeck: float = 0.03  # [m/s] (A)
    v_align: float = 0.005  # [m/s] below it a contact sticks (A)
    perp_ratio: float = 0.0  # mu across the slip while sliding: 0 is a friction circle
    stick_perp_ratio: float = 0.3  # mu across the expected load while sticking: holds within 4.4 % of mu_s
    mu_noise: float = 0.2  # spatial mu variation (A)
    mu_noise_length: float = 0.3  # [m] (A)
    # [rad/s] rolling resistance and bulldozing fade in over rr_w0 x radius of hub speed, 7.5 mm/s (A; design
    # spec 6.2 has 0.2, but over 3 cm/s a rover dug in by the strong preset creeps round at 0.25 x the fresh
    # rate instead of sticking; at 0.05 it turns 0.07 x, measured).
    rr_w0: float = 0.05
    # Dig-in on loose ground (design spec 6.5, D21); how strong is the ground's (terrains.DIG, the one switch:
    # strong by default, the user's choice, Q12).
    dig: bool = True
    dig_heal_length: float = 0.3  # [m] one wheel diameter of travel heals a dug wheel by 1/e (A)
    default_surface: str = "regolith"  # terrains.TYPES key: ground where the world has no ground map
    object_surface: str = "manmade"  # terrains.TYPES key: objects whose SDF sets no friction
    # Wheel joints: DART never clamps the torque, and the velocity limit (1.5 x the 10.8 rad/s free speed)
    # never acts as a hidden brake (design spec 6.2).
    wheel_effort: float = 1000.0  # [N m]
    wheel_velocity: float = 16.0  # [rad/s]
    odom_rate: float = 50.0  # [Hz]
    state_rate: float = 50.0  # [Hz] DRIVETRAIN_TOPIC
    # Dust behind the rear wheels (design spec 6.5, D15): particles per second = dust factor x
    # (speed_gain x hub speed + slip_gain x slip speed) x dig factor, at most dust_max (A gains).
    dust_rate: float = 10.0  # [Hz] commands to the emitters
    dust_speed_gain: float = 8.0  # [1/m]
    dust_slip_gain: float = 25.0  # [1/m]
    dust_max: float = 40.0  # [1/s]
    dust_min_speed: float = 0.05  # [m/s]
    # The emitter (M: tuned in the render research, design spec 6.5): a box at the ground behind each rear
    # wheel blowing back and up; the colour is the catalogue's dust colour, fading out.
    dust_box: float = 0.25  # [m]
    dust_particle: float = 0.2  # [m]
    dust_lifetime: float = 1.6  # [s]
    dust_speed: tuple[float, float] = (0.15, 0.5)  # [m/s]
    dust_alpha: float = 0.28
    dust_pitch: float = 0.6  # [rad] above the horizontal, backwards (A)


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
    # 1280x720: at 640x480 a 20 cm ArUco face read only to ~2.5 m (sim/README.md, Known limitations). The
    # depth image has the same size (one RGB-D sensor).
    camera_size: tuple[int, int] = (1280, 720)
    camera_hfov: float = 1.5  # [rad]
    camera_clip: tuple[float, float] = (0.1, 40.0)  # [m] depth range
    # The RGB sees the far field to 80 km while the depth stays clipped at camera_clip (design spec 7, D14). No
    # SDF <noise> (design spec 4's stddev 0.06): on an rgbd_camera it aborts gz on Metal (measured: Ogre
    # RenderingAPIException, float4 output to an RGBA32Uint attachment), so noise belongs in the station.
    camera_far: float = 80_000.0  # [m] RGB far clip
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
    drive: DriveParams = DriveParams()
    # Tyre compliance (design spec 6.7, D18; phase 2, off): a 0.1 kg hub between rocker and wheel on two sprung
    # prismatic joints, axial then radial, each within +-tire_travel. It turns 400-480 Hz contact chatter into a
    # 12-16 Hz wheel hop (M); revisit when the wheel type is known (Q8).
    tire_compliance: bool = False
    tire_radial: tuple[float, float] = (60_000.0, 170.0)  # [N/m], [N s/m] (M: prototype)
    tire_axial: tuple[float, float] = (40_000.0, 40.0)  # [N/m], [N s/m] (M: prototype)
    tire_travel: float = 0.03  # [m]
    hub_mass: float = 0.1  # [kg]


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
    sdf.camera(sensor, p.camera_hfov, p.camera_size, (p.camera_clip[0], p.camera_far), depth_clip=p.camera_clip)

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
    _add_dust_emitter(link, p, f"dust_r{side[0]}")


def _add_dust_emitter(link, p, name):
    """A particle emitter at the ground behind the rocker's rear wheel (on the rocker: a wheel link spins),
    idle until the physical drivetrain sets its rate on DUST_TOPIC (design spec 6.5, D15). It starts not
    emitting (SDF's default is to emit); scatter ratio 0 is meant to keep its particles out of the depth
    image and point cloud (Q11), but gz-rendering 8 ignores it (measured: tests/test_render.py). Each particle
    is the soft puff sprite DUST_SPRITE, tinted by the colours (M: the render research's tuned plume)."""
    from urc import terrains  # here: the catalogue imports the texture generators
    d = p.drive
    emitter = sdf.sub(link, "particle_emitter", name=name, type="box")
    behind = p.wheel_dx + p.wheel_radius + d.dust_box / 4
    sdf.pose(emitter, (-behind, 0, p.wheel_dz - p.wheel_radius + d.dust_box / 2, 0, -d.dust_pitch, math.pi))
    sdf.sub(emitter, "emitting", False)
    sdf.sub(emitter, "size", (d.dust_box,) * 3)
    sdf.sub(emitter, "particle_size", (d.dust_particle,) * 3)
    sdf.sub(emitter, "lifetime", d.dust_lifetime)
    sdf.sub(emitter, "rate", 0.0)
    sdf.sub(emitter, "min_velocity", d.dust_speed[0])
    sdf.sub(emitter, "max_velocity", d.dust_speed[1])
    sdf.sub(emitter, "scale_rate", 1.0)
    sdf.sub(emitter, "color_start", (*terrains.DUST_RGB, d.dust_alpha))
    sdf.sub(emitter, "color_end", (*terrains.DUST_RGB, 0.0))
    sdf.sub(emitter, "topic", DUST_TOPIC.format(link=link.get("name"), emitter=name))
    sdf.sub(emitter, "particle_scatter_ratio", 0.0)
    sdf.sub(sdf.sub(sdf.sub(sdf.sub(emitter, "material"), "pbr"), "metal"), "albedo_map",
            sdf.model_uri(MODEL_DIR.name, DUST_SPRITE))


def _add_wheel(model, p, side, name, sign, ahead):
    xyz = (ahead * p.wheel_dx, sign * p.pivot_y, p.pivot_z + p.wheel_dz)
    link = sdf.link(model, name, (*xyz, 0, 0, 0), p.wheel_mass,
                    wheel_inertia(p.wheel_mass, p.wheel_radius, p.wheel_width))
    tire, pose = sdf.cylinder(p.wheel_radius, p.wheel_width), (0, 0, 0, math.pi / 2, 0, 0)
    if _physical(p):
        # Isotropic: the drivetrain sets the friction of every wheel contact itself.
        sdf.collision(link, "tire", tire, pose, mu=1.0)
        effort, velocity = p.drive.wheel_effort, p.drive.wheel_velocity
    else:
        # A cylinder runs along its z axis; roll it onto the axle. fdir1 is the
        # axle in the collision frame: mu acts along it (lateral), mu2 across it
        # (longitudinal).
        sdf.collision(link, "tire", tire, pose, mu=p.mu_lateral, mu2=p.mu_longitudinal, fdir1=(0, 0, 1))
        effort, velocity = p.wheel_effort, p.wheel_speed
    _visual(link, "tire", tire, pose, TIRE_COLOR)
    parent = _add_tire_hubs(model, p, f"rocker_{side}", name, xyz) if p.tire_compliance else f"rocker_{side}"
    sdf.joint(model, f"{name}_joint", "revolute", parent, name, (0, 1, 0), -1e16, 1e16, effort=effort,
              velocity=velocity)


def _add_tire_hubs(model, p, rocker, wheel, xyz):
    """Tyre compliance (Params.tire_compliance): the rocker carries the wheel through two hub links on sprung
    prismatic joints, along the axle, then radially; returns the link the wheel joint hangs from."""
    parent = rocker
    for tag, axis, (stiffness, damping) in (("axial", (0, 1, 0), p.tire_axial), ("radial", (0, 0, 1), p.tire_radial)):
        hub = f"{wheel}_hub_{tag}"
        sdf.link(model, hub, (*xyz, 0, 0, 0), p.hub_mass, (1e-4,) * 3)
        sdf.joint(model, f"{wheel}_tire_{tag}", "prismatic", parent, hub, axis, -p.tire_travel, p.tire_travel,
                  damping=damping, stiffness=stiffness)
        parent = hub
    return parent


def _physical(p):
    if p.drive.mode not in ("diffdrive", "physical"):
        raise ValueError(f"DriveParams.mode {p.drive.mode!r}: 'diffdrive' or 'physical'")
    return p.drive.mode == "physical"


def _add_plugins(model, p):
    sdf.plugin(model, "RockerDifferential", "rover_sim::RockerDifferential", left_joint="rocker_left_joint",
               right_joint="rocker_right_joint", stiffness=p.diff_stiffness, damping=p.diff_damping)
    if _physical(p):
        _add_drivetrain(model, p)  # never beside DiffDrive: it would override the torques (D22)
    else:
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


def _add_drivetrain(model, p):
    """plugins/rover_drivetrain.cpp, set from DriveParams (design spec 6.2)."""
    d = p.drive
    plugin = sdf.plugin(model, "RoverDrivetrain", "rover_sim::RoverDrivetrain", topic=CMD_VEL_TOPIC,
                        cmd_timeout=d.cmd_timeout, cmd_timeout_clock=d.cmd_timeout_clock, odom_topic=ODOM_TOPIC,
                        tf_topic=TF_TOPIC, frame_id="odom", child_frame_id="base_link",
                        odom_publish_frequency=d.odom_rate, state_topic=DRIVETRAIN_TOPIC, state_rate=d.state_rate,
                        dust_rate=d.dust_rate, track=2 * p.pivot_y, radius=p.wheel_radius,
                        track_multiplier=d.track_multiplier)
    for side, s, _ in SIDES:
        for _, e, _ in ENDS:
            sdf.group(plugin, "wheel", name=f"{e}{s}", joint=f"wheel_{e}{s}_joint", link=f"wheel_{e}{s}", side=side)
    for side, s, _ in SIDES:
        sdf.group(plugin, "dust", wheel=f"wheel_r{s}", topic=DUST_TOPIC.format(link=f"rocker_{side}",
                                                                                emitter=f"dust_r{s}"))
    sdf.group(plugin, "motor", voltage=d.voltage, resistance=d.resistance, kt=d.kt, ke=d.ke, gear=d.gear,
              efficiency=d.efficiency, rotor_inertia=d.rotor_inertia, free_current=d.free_current,
              output_friction=d.output_friction, current_limit=d.current_limit)
    sdf.group(plugin, "driveline", stiffness=d.driveline_stiffness, damping=d.driveline_damping, backlash=d.backlash)
    sdf.group(plugin, "controller", kp=d.kp, ki=d.ki, speed_filter=d.speed_filter, accel=d.accel,
              max_speed=p.wheel_speed, substeps=d.substeps)
    contact = sdf.group(plugin, "contact", v_stribeck=d.v_stribeck, v_align=d.v_align, perp_ratio=d.perp_ratio,
                        stick_perp_ratio=d.stick_perp_ratio, mu_noise=d.mu_noise, mu_noise_length=d.mu_noise_length,
                        rr_w0=d.rr_w0, dig=d.dig, dig_heal_length=d.dig_heal_length, default_surface=d.default_surface,
                        object_surface=d.object_surface)
    for row in surface_rows(d.default_surface, d.object_surface):
        sdf.group(contact, "surface", **row)
    sdf.group(plugin, "dust_rule", speed_gain=d.dust_speed_gain, slip_gain=d.dust_slip_gain, max_rate=d.dust_max,
              min_speed=d.dust_min_speed)


def surface_rows(*keys):
    """The traction of catalogue ground types (terrains.TYPES, design spec 5.6, under the dig-in preset
    terrains.DIG, as worlds write it) as the drivetrain's <surface> rows, what it uses where the world has no
    ground map. A key the catalogue lacks is left out; the drivetrain reports it when it starts and grips
    there as plain Coulomb mu 1."""
    from urc import terrains  # here: the catalogue imports the texture generators
    rows = []
    for key in dict.fromkeys(keys):
        kind = terrains.TYPES.get(key)
        if kind is not None:
            rows.append(dict(key=key, **dataclasses.asdict(terrains.traction(kind)), dust=kind.appearance.dust))
    return rows


def __getattr__(name):
    """The viewer names (VIEWER_NAMES) from viewers.py, imported on first use:
    viewers imports this module for Params."""
    if name in VIEWER_NAMES:
        import viewers
        return getattr(viewers, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def main():
    from urc import textures  # here: only main writes files
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    out = MODEL_DIR / "model.sdf"
    out.write_text(build_sdf(Params()))
    sprite = MODEL_DIR / DUST_SPRITE
    sprite.parent.mkdir(parents=True, exist_ok=True)
    textures.dust_puff(sprite)
    print(f"wrote {out} and {sprite.relative_to(MODEL_DIR)}")
    import viewers  # here, not at the top: viewers imports this module
    viewers.write_all(MODELS_DIR)


if __name__ == "__main__":
    main()
