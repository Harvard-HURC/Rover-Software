"""The driver station's cameras (sim/station), generated with the rover by
`pixi run sim-model` (gen_model.main calls write_all): the chase camera that
follows the rover, sim/models/chase_camera, and the rover eye at its camera
pivot, sim/models/eye_camera.

Each is a model of its own, which the station spawns into a running world
(station/link.py) and the ChaseCamera plugin (plugins/chase_camera.cpp)
moves. Every dimension lives in ChaseParams and EyeParams; the eye takes its
mount, start and limits from the rover's pan-tilt head (gen_model.Params).
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


def _viewer_sdf(name, topic, rate, size, hfov, clip, plugin):
    """A driver station camera as an SDF 1.11 document: one link without
    gravity or collisions (nothing changes its velocity, which stays zero, so
    only the ChaseCamera plugin's teleports move it) carrying an RGB camera,
    and the plugin with the (tag, value) elements in `plugin`."""
    root, model = sdf.model_root(name)
    link = sdf.link(model, "link")
    sdf.sub(link, "gravity", False)
    sdf.inertial(link, 0.1, sdf.box_inertia(0.1, (0.1, 0.1, 0.1)))
    sensor = sdf.sub(link, "sensor", name="camera", type="camera")
    sdf.sub(sensor, "always_on", True)
    sdf.sub(sensor, "update_rate", rate)
    sdf.sub(sensor, "topic", topic)
    sdf.camera(sensor, hfov, size, clip, image_format="R8G8B8")
    sdf.plugin(model, "ChaseCamera", "rover_sim::ChaseCamera", **dict(plugin))
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


def write_all(models_dir):
    """Write every viewer model into models_dir/<name>/{model.sdf, model.config}."""
    cameras = ((CHASE_MODEL, build_chase_sdf(ChaseParams()),
                "Third-person camera for the driver station (sim/station), moved by the ChaseCamera plugin "
                "(edit ChaseParams)."),
               (EYE_MODEL, build_eye_sdf(EyeParams(), Params()),
                "First-person camera for the driver station (sim/station): the ChaseCamera plugin keeps it at "
                "the rover's camera pivot (edit EyeParams)."))
    for name, text, description in cameras:
        directory = Path(models_dir) / name
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "model.sdf").write_text(text)
        (directory / "model.config").write_text(sdf.model_config(name, description, "sim/gen_model.py"))
        print(f"wrote {directory / 'model.sdf'}")
