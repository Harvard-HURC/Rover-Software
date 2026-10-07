"""The driver station's cameras (sim/station), generated with the rover by
`pixi run sim-model` (gen_model.main calls write_all): the chase camera that
follows the rover, sim/models/chase_camera; the rover eye at its camera
pivot, sim/models/eye_camera; and the free-flying inspection camera,
sim/models/fly_camera.

Each is a model of its own, which the station spawns into a running world
(station/link.py). The ChaseCamera plugin (plugins/chase_camera.cpp) moves
the chase camera and the eye, the FlyCamera plugin (plugins/fly_camera.cpp)
the fly camera; the offline map tool (tools/render_map.py) flies a variant of
it. Every dimension lives in ChaseParams, EyeParams and FlyParams; the eye
takes its mount, start and limits from the rover's pan-tilt head
(gen_model.Params).
"""
from dataclasses import dataclass
from pathlib import Path

from gen_model import Params
from urc import sdf  # the SDF writer every generated model shares

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
# Fly camera (plugins/fly_camera.cpp has the details; the station publishes,
# never calls services: blocking Python requests stall ~1 s under a busy
# subscriber, README "Gazebo lessons").
FLY_MODEL = "fly_camera"
FLY_IMAGE_TOPIC = "/fly_camera/image"
# gz.msgs.Twist: linear forward, left, up in cruise speeds (each within
# +-FlyParams.fast), angular.z yaw rate (left +), angular.y pitch rate (down +)
# [rad/s]; held for FlyParams.deadman.
FLY_CMD_TOPIC = "/fly_camera/cmd"
FLY_SPEED_TOPIC = "/fly_camera/speed"  # gz.msgs.Double: speed multiplier, within FlyParams.speed_scales
FLY_LOOK_TOPIC = "/fly_camera/look"  # gz.msgs.Vector3d: d yaw, d pitch [rad], smoothed
# gz.msgs.Pose: fly there (smoothstep, 0.4-1.5 s; header data key FLY_JUMP_KEY:
# at once); the camera looks along the pose's x axis, roll dropped.
FLY_GOTO_TOPIC = "/fly_camera/goto"
FLY_JUMP_KEY = "jump"
FLY_MODE_TOPIC = "/fly_camera/mode"  # gz.msgs.StringMsg: one of FLY_MODES, or "ortho <width [m]>"
FLY_MODES = ("free", "follow", "top", "level", "stop", "ortho", "perspective")
# gz.msgs.StringMsg, JSON at 10 Hz of sim time: t, mode, x, y, z, yaw, pitch,
# agl, ground, speed (multiplier), v [m/s], ortho (window width, 0 perspective), goto
FLY_STATE_TOPIC = "/fly_camera/state"


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


@dataclass(frozen=True)
class FlyParams:
    """The driver station's free inspection camera (plugins/fly_camera.cpp):
    flown by velocity commands, kept above the world's heightmap, with
    top-down and orthographic views. Values from the realism design's section
    8.1, which took them from the measured prototype
    (sim/data/research/flycam/prototype/flycam_proto.cpp), unless marked (A)."""
    target: str = "rover"  # model that follow mode moves with
    clearance: float = 1.0  # [m] above the ground (A)
    time_constant: float = 0.2  # [s] first-order lag of the velocity
    # [s] of the view: frame-to-frame look motion varied 4.8 % against 29 %
    # unsmoothed (measurements/look_smoothing_tau_0.08.json, ortho_look_pause_checks.json)
    look_time_constant: float = 0.08
    deadman: float = 0.3  # [s] a command older than this stops the camera (A: 6 station ticks of 50 ms)
    # wall: a stalled world cannot keep an old command alive; sim: tests at
    # real-time factor 0, where wall time means nothing.
    deadman_clock: str = "wall"
    # Cruise speed = height above the ground x speed_per_agl, within speed_limits:
    # the picture moves about as fast at every height (A).
    speed_per_agl: float = 1.0  # [1/s]
    speed_limits: tuple[float, float] = (2.0, 200.0)  # [m/s]
    fast: float = 4.0  # cruise multiple while the fast key is held (A)
    speed_scales: tuple[float, float] = (0.25, 4.0)  # the speed multiplier's range (A)
    max_altitude: float = 2000.0  # [m] above the highest terrain (A)
    margin: float = 100.0  # [m] the camera may go beyond the terrain edge (A)
    rate: float = 20.0  # [Hz]
    size: tuple[int, int] = (1280, 720)  # gate G6: 18.5 fps with the full stand-in config (gates.json)
    hfov: float = 1.2  # [rad]
    clip: tuple[float, float] = (0.1, 80_000.0)  # [m] far: the far-field ring (decision D14)


def _viewer_sdf(name, topic, rate, size, hfov, clip, plugin, system="ChaseCamera"):
    """A driver station camera as an SDF 1.11 document: one link without
    gravity or collisions (nothing changes its velocity, which stays zero, so
    only its plugin's pose commands move it) carrying an RGB camera, and the
    plugin rover_sim::<system> with the (tag, value) elements in `plugin`."""
    root, model = sdf.model_root(name)
    link = sdf.link(model, "link")
    sdf.sub(link, "gravity", False)
    sdf.inertial(link, 0.1, sdf.box_inertia(0.1, (0.1, 0.1, 0.1)))
    sensor = sdf.sub(link, "sensor", name="camera", type="camera")
    sdf.sub(sensor, "always_on", True)
    sdf.sub(sensor, "update_rate", rate)
    sdf.sub(sensor, "topic", topic)
    sdf.camera(sensor, hfov, size, clip, image_format="R8G8B8")
    sdf.plugin(model, system, f"rover_sim::{system}", **dict(plugin))
    return sdf.document(root)


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


def build_fly_sdf(f: FlyParams) -> str:
    """The fly camera as an SDF 1.11 document; it starts where it is spawned."""
    return _viewer_sdf(FLY_MODEL, FLY_IMAGE_TOPIC, f.rate, f.size, f.hfov, f.clip, (
        ("target", f.target), ("clearance", f.clearance),
        ("time_constant", f.time_constant), ("look_time_constant", f.look_time_constant),
        ("deadman", f.deadman), ("deadman_clock", f.deadman_clock),
        ("speed_per_agl", f.speed_per_agl), ("min_speed", f.speed_limits[0]), ("max_speed", f.speed_limits[1]),
        ("fast", f.fast), ("min_scale", f.speed_scales[0]), ("max_scale", f.speed_scales[1]),
        ("max_altitude", f.max_altitude), ("margin", f.margin),
        ("cmd_topic", FLY_CMD_TOPIC), ("speed_topic", FLY_SPEED_TOPIC), ("look_topic", FLY_LOOK_TOPIC),
        ("goto_topic", FLY_GOTO_TOPIC), ("mode_topic", FLY_MODE_TOPIC), ("state_topic", FLY_STATE_TOPIC)),
        system="FlyCamera")


def write_all(models_dir):
    """Write every viewer model into models_dir/<name>/{model.sdf, model.config}."""
    cameras = ((CHASE_MODEL, build_chase_sdf(ChaseParams()),
                "Third-person camera for the driver station (sim/station), moved by the ChaseCamera plugin "
                "(edit ChaseParams)."),
               (EYE_MODEL, build_eye_sdf(EyeParams(), Params()),
                "First-person camera for the driver station (sim/station): the ChaseCamera plugin keeps it at "
                "the rover's camera pivot (edit EyeParams)."),
               (FLY_MODEL, build_fly_sdf(FlyParams()),
                "Free inspection camera for the driver station (sim/station), flown by the FlyCamera plugin "
                "(edit FlyParams)."))
    for name, text, description in cameras:
        directory = Path(models_dir) / name
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "model.sdf").write_text(text)
        (directory / "model.config").write_text(sdf.model_config(name, description, "sim/gen_model.py"))
        print(f"wrote {directory / 'model.sdf'}")
