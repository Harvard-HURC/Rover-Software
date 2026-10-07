"""Gazebo models for the URC missions.

Each builder writes a model directory under models_dir (model.sdf +
model.config) and returns the model name, so worlds include it as
model://<name>. Shared textures and meshes come from media.Media (urc_media).

Frames: x forward, y left, z up; the origin is on the ground under the
object (free objects: on the surface they rest on).
"""
import math

import numpy as np

from . import rules, sdf, textures

STEEL = (0.62, 0.63, 0.66)
ALUMINIUM = (0.75, 0.76, 0.78)
DARK = (0.12, 0.12, 0.13)
ORANGE = (0.95, 0.42, 0.05)
WHITE = (0.95, 0.95, 0.93)
RED = (0.75, 0.08, 0.06)
BLUE = (0.10, 0.25, 0.75)
YELLOW = (0.95, 0.78, 0.08)
OLIVE = (0.38, 0.40, 0.22)


def picture(link, media, name, image_uri, width, height, xyzrpy, emissive=False):
    """A flat textured rectangle (visual only) facing the +x of its pose."""
    sdf.visual(link, name, sdf.mesh(media.quad(), (1, width, height)), xyzrpy, albedo=image_uri,
               emissive=(0.4, 0.4, 0.4) if emissive else None, emissive_map=image_uri if emissive else None,
               roughness=1.0)


def handle_bar(link, name, length, diameter, center, axis="y", posts=0.04, color=DARK):
    """A grab bar on two posts, `posts` metres above `center` (the surface)."""
    x, y, z = center
    along = {"x": (1, 0, 0), "y": (0, 1, 0)}[axis]
    rpy = (0, math.pi / 2, 0) if axis == "x" else (math.pi / 2, 0, 0)
    sdf.shape(link, f"{name}_bar", sdf.cylinder(diameter / 2, length), (x, y, z + posts, *rpy), color)
    for k, s in enumerate((-1, 1)):
        px, py = x + s * along[0] * (length / 2 - 0.01), y + s * along[1] * (length / 2 - 0.01)
        sdf.shape(link, f"{name}_post{k}", sdf.cylinder(0.006, posts), (px, py, z + posts / 2), color)


def monitor(model, rate=10):
    """Publish the model's joint positions and link world poses on
    /model/<name>/state for the referee (sim/plugins/joint_monitor.cpp)."""
    sdf.plugin(model, "JointMonitor", "rover_sim::JointMonitor", update_rate=rate)


def _box_link(model, name, size, mass, color, xyzrpy=(0, 0, 0), mu=None):
    link = sdf.link(model, name, xyzrpy, mass, sdf.box_inertia(mass, size), com=(0, 0, size[2] / 2))
    sdf.shape(link, "body", sdf.box(size), (0, 0, size[2] / 2), color, mu=mu)
    return link


# --- Autonomy ----------------------------------------------------------------------

def ar_post(models_dir, media, tag_id, center_height=1.0):
    """URC AR post (rule 1.e.xii): the same tag on three 20 x 20 cm faces
    around a pole, faces centred `center_height` above the ground."""
    assert rules.AR_MIN_HEIGHT + rules.AR_FACE / 2 <= center_height <= rules.AR_MAX_HEIGHT - rules.AR_FACE / 2
    name = f"urc_ar_post_{tag_id}"
    root, model = sdf.model_root(name, static=True)
    link = sdf.link(model, "link")
    face = rules.AR_FACE
    top = center_height + face / 2 + 0.03
    sdf.shape(link, "pole", sdf.cylinder(0.022, top), (0, 0, top / 2), ALUMINIUM, metalness=0.6, roughness=0.4)
    sdf.shape(link, "foot", sdf.cylinder(0.15, 0.02), (0, 0, 0.01), DARK)
    # The faces form a triangular prism around the pole.
    inradius = face / (2 * math.sqrt(3))
    sdf.collision(link, "marker", sdf.cylinder(face / math.sqrt(3), face), (0, 0, center_height))
    for k in range(rules.AR_FACES):
        yaw = 2 * math.pi * k / rules.AR_FACES
        c, s = math.cos(yaw), math.sin(yaw)
        sdf.visual(link, f"backing{k}", sdf.box((0.004, face, face)),
                   (inradius * c, inradius * s, center_height, 0, 0, yaw), WHITE)
        d = inradius + 0.0025
        picture(link, media, f"tag{k}", media.aruco(tag_id), face, face, (d * c, d * s, center_height, 0, 0, yaw),
                emissive=True)
    sdf.write_model(models_dir, name, root, f"URC AR post with ArUco 4x4_50 tag {tag_id} on three sides.")
    return name


# Astronaut joints: name -> (parent, child, axis in the child frame). Positive
# shoulder pitch raises the arm forward, positive roll raises it sideways
# (outwards) and positive elbow bends the forearm forward and up.
def _arm_joints(side, sign):
    return [
        (f"shoulder_{side}_pitch", "body", f"shoulder_{side}", (0, -1, 0)),
        (f"shoulder_{side}_roll", f"shoulder_{side}", f"upper_arm_{side}", (sign, 0, 0)),
        (f"elbow_{side}", f"upper_arm_{side}", f"forearm_{side}", (0, -1, 0)),
    ]


ASTRONAUT_JOINTS = [j[0] for side, sign in (("left", 1), ("right", -1)) for j in _arm_joints(side, sign)]
SHOULDER_HEIGHT = 1.38
UPPER_ARM = 0.30
FOREARM = 0.30
# Arm poses (joint name -> angle [rad]) the referee and static figures use.
POSE_REST = {j: 0.0 for j in ASTRONAUT_JOINTS}
POSE_SHOW_SIGN = dict(POSE_REST, shoulder_left_pitch=math.radians(40), elbow_left=math.radians(140))


def _rot(axis, angle):
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def astronaut(models_dir, media, name="urc_astronaut", sign_texture=None, static=False, arm_pose=None):
    """An astronaut in an EVA suit (rule 1.e.iv) with jointed arms and a sign
    in the left hand (rule 1.e.vi lets teams choose the sign's image).

    Dynamic (static=False): links ignore gravity, the referee moves the whole
    figure with set_pose and the arms with JointPositionController topics
    /model/<model>/joint/<joint>/0/cmd_pos. Static: a figure posed by arm_pose.
    """
    arm_pose = dict(POSE_REST, **(arm_pose or {}))
    sign_texture = sign_texture or media.aruco(10)
    root, model = sdf.model_root(name, static=static)
    suit, trim, visor = (0.93, 0.93, 0.9), ORANGE, (0.55, 0.42, 0.1)
    gravity = static  # dynamic figures float where the referee puts them

    body = sdf.link(model, "body", (0, 0, 0), 60.0, sdf.box_inertia(60.0, (0.3, 0.45, 1.7)), com=(0, 0, 0.95),
                    gravity=gravity)
    for side, y in (("left", 0.1), ("right", -0.1)):
        sdf.shape(body, f"leg_{side}", sdf.cylinder(0.085, 0.86), (0, y, 0.43), suit)
        sdf.shape(body, f"boot_{side}", sdf.box((0.26, 0.12, 0.08)), (0.04, y, 0.04), DARK)
    sdf.shape(body, "torso", sdf.box((0.26, 0.44, 0.58)), (0, 0, 1.15), suit)
    sdf.shape(body, "belt", sdf.box((0.27, 0.45, 0.06)), (0, 0, 0.88), trim, collide=False)
    sdf.shape(body, "backpack", sdf.box((0.16, 0.38, 0.46)), (-0.21, 0, 1.18), WHITE)
    sdf.shape(body, "helmet", sdf.sphere(0.16), (0, 0, 1.62), suit)
    sdf.visual(body, "visor", sdf.sphere(0.13), (0.06, 0, 1.63), visor, metalness=0.8, roughness=0.2)

    def transform(xyz=(0, 0, 0), axis=(1, 0, 0), angle=0.0):
        T = np.eye(4)
        T[:3, :3] = _rot(axis, angle)
        T[:3, 3] = xyz
        return T

    # Forward kinematics: each joint sits at its child link's origin, so a
    # child frame is the parent frame, moved to the joint, turned by the angle.
    frames = {}
    for side, sign in (("left", 1), ("right", -1)):
        (pitch, _, _, a_pitch), (roll, _, _, a_roll), (elbow, _, _, a_elbow) = _arm_joints(side, sign)
        frames[f"shoulder_{side}"] = transform((0, sign * 0.27, SHOULDER_HEIGHT), a_pitch, arm_pose[pitch])
        frames[f"upper_arm_{side}"] = frames[f"shoulder_{side}"] @ transform(axis=a_roll, angle=arm_pose[roll])
        frames[f"forearm_{side}"] = frames[f"upper_arm_{side}"] @ transform((0, 0, -UPPER_ARM), a_elbow,
                                                                            arm_pose[elbow])

    def posed_link(link_name, mass, size):
        T = frames[link_name]
        xyzrpy = (*T[:3, 3], *sdf.matrix_to_rpy(T[:3, :3]))
        return sdf.link(model, link_name, xyzrpy, mass, sdf.box_inertia(mass, size), gravity=gravity)

    for side, sign in (("left", 1), ("right", -1)):
        posed_link(f"shoulder_{side}", 0.2, (0.05, 0.05, 0.05))
        upper = posed_link(f"upper_arm_{side}", 2.0, (0.1, 0.1, UPPER_ARM))
        sdf.shape(upper, "arm", sdf.cylinder(0.055, UPPER_ARM), (0, 0, -UPPER_ARM / 2), suit)
        sdf.shape(upper, "shoulder", sdf.sphere(0.07), (0, 0, 0), suit, collide=False)
        fore = posed_link(f"forearm_{side}", 1.5, (0.1, 0.1, FOREARM))
        sdf.shape(fore, "arm", sdf.cylinder(0.05, FOREARM), (0, 0, -FOREARM / 2), suit)
        sdf.shape(fore, "glove", sdf.sphere(0.06), (0, 0, -FOREARM - 0.03), trim)
        if side == "left":
            # Held at the glove: facing backwards with the arm down, forwards at
            # chest height in POSE_SHOW_SIGN (forearm turned up through 180 deg).
            sdf.visual(fore, "sign_board", sdf.box((0.01, 0.36, 0.36)), (0, 0, -FOREARM + 0.05), WHITE)
            picture(fore, media, "sign", sign_texture, 0.34, 0.34, (-0.006, 0, -FOREARM + 0.05, 0, 0, math.pi),
                    emissive=True)
        for joint_name, parent, child, axis in _arm_joints(side, sign):
            if child.startswith("forearm"):
                limits = (-0.1, 2.6)
            elif "roll" in joint_name:
                limits = (-0.3, 2.8)
            else:
                limits = (-1.0, 3.1)
            sdf.joint(model, joint_name, "revolute", parent, child, axis, *limits, effort=50, velocity=4,
                      damping=2.0)
            if not static:
                sdf.plugin(model, "gz-sim-joint-position-controller-system",
                           "gz::sim::systems::JointPositionController", joint_name=joint_name,
                           use_velocity_commands=True, p_gain=6.0, cmd_max=4.0, cmd_min=-4.0,
                           initial_position=arm_pose[joint_name])
    sdf.write_model(models_dir, name, root, "Astronaut in an EVA suit with jointed arms and a sign.")
    return name


def rock_pick_hammer(models_dir):
    """Rock pick hammer for Fetch! (rule 1.e.viii): Estwing E3-22P-like,
    33 cm long, 18 cm head, blue grip, lying on its side."""
    name = "urc_rock_pick_hammer"
    root, model = sdf.model_root(name)
    length, head, mass = 0.33, 0.18, 0.9
    link = sdf.link(model, "link", None, mass, sdf.box_inertia(mass, (length, 0.06, 0.03)), com=(0.08, 0, 0.0125))
    z = 0.0125
    sdf.shape(link, "grip", sdf.box((0.14, 0.032, 0.025)), (-0.09, 0, z), BLUE, mu=0.9)
    sdf.shape(link, "shaft", sdf.box((0.15, 0.022, 0.018)), (0.05, 0, z), STEEL, metalness=0.7, roughness=0.35)
    # Head across the handle: square hammer face on one side, a pick on the other.
    x = length / 2 - 0.0125
    sdf.shape(link, "head", sdf.box((0.026, 0.09, 0.025)), (x, -0.045 + 0.02, z), STEEL, metalness=0.7,
              roughness=0.35)
    sdf.shape(link, "face", sdf.cylinder(0.014, 0.02), (x, -0.035, z, math.pi / 2, 0, 0), STEEL, metalness=0.7,
              roughness=0.35)
    sdf.shape(link, "pick", sdf.box((0.018, head - 0.09, 0.012)), (x, 0.065 - 0.005, z), STEEL, metalness=0.7,
              roughness=0.35)
    monitor(model)
    sdf.write_model(models_dir, name, root, "Rock pick hammer (Fetch! task).")
    return name


# --- Course infrastructure ------------------------------------------------------------

ANTENNA_OFFSET = (-1.0, -3.5)  # mast base from the C2 model origin [m], within 5 m (2.c.iv)


def c2_station(models_dir):
    """Command and control station (rule 3.d.i): the back of a moving truck,
    plus the team's 3 m antenna mast (rule 2.c.iv)."""
    name = "urc_c2_station"
    root, model = sdf.model_root(name, static=True)
    link = sdf.link(model, "link")
    sdf.shape(link, "box", sdf.box((4.2, 2.4, 2.3)), (0, 0, 0.55 + 1.15), (0.92, 0.9, 0.85))
    sdf.shape(link, "stripe", sdf.box((4.22, 2.42, 0.3)), (0, 0, 1.0), ORANGE, collide=False)
    sdf.shape(link, "chassis", sdf.box((4.0, 2.0, 0.25)), (0, 0, 0.45), DARK)
    for k, (x, y) in enumerate(((-1.4, 1.0), (-1.4, -1.0), (0.2, 1.0), (0.2, -1.0))):
        sdf.shape(link, f"wheel{k}", sdf.cylinder(0.38, 0.25), (x, y, 0.38, math.pi / 2, 0, 0), DARK)
    ax, ay = ANTENNA_OFFSET
    h = rules.ANTENNA_MAX_HEIGHT
    sdf.shape(link, "mast", sdf.cylinder(0.025, h - 0.3), (ax, ay, (h - 0.3) / 2), ALUMINIUM)
    sdf.shape(link, "antenna", sdf.cylinder(0.02, 0.3), (ax, ay, h - 0.15), DARK, collide=False)
    sdf.shape(link, "mast_foot", sdf.box((0.6, 0.6, 0.04)), (ax, ay, 0.02), DARK)
    sdf.write_model(models_dir, name, root, "URC command and control station with antenna mast.")
    return name


def landing_pad(models_dir, media):
    name = "urc_landing_pad"
    root, model = sdf.model_root(name, static=True)
    link = sdf.link(model, "link")
    image = media.texture("landing_pad", textures.label_image, "H", (256, 256), (60, 60, 64), (250, 250, 250))
    sdf.shape(link, "pad", sdf.box((1.5, 1.5, 0.03)), (0, 0, 0.015), (0.24, 0.24, 0.25))
    picture(link, media, "mark", image, 1.4, 1.4, (0, 0, 0.0315, 0, -math.pi / 2, 0))
    sdf.write_model(models_dir, name, root, "Drone landing pad (rule 2.b.iv).")
    return name


def start_gate(models_dir, media, width=3.0):
    name = "urc_start_gate"
    root, model = sdf.model_root(name, static=True)
    link = sdf.link(model, "link")
    banner = media.sign("start", ["START"], size=(1024, 256), bg=(240, 120, 20), fg=(255, 255, 255),
                        border=(255, 255, 255))
    for k, y in enumerate((width / 2, -width / 2)):
        sdf.shape(link, f"pole{k}", sdf.cylinder(0.04, 2.4), (0, y, 1.2), ORANGE)
    sdf.visual(link, "banner_board", sdf.box((0.01, width, 0.5)), (0, 0, 2.1), WHITE)
    for k, yaw in enumerate((0, math.pi)):
        x = 0.006 if k == 0 else -0.006
        picture(link, media, f"banner{k}", banner, width, 0.5, (x, 0, 2.1, 0, 0, yaw))
    sdf.write_model(models_dir, name, root, "Start gate.")
    return name


def field_sign(models_dir, media, key, lines, height=1.0):
    """A sign on a post (rule 1.c.iii: read signs placed in the field), readable
    from +x."""
    name = f"urc_sign_{key}"
    root, model = sdf.model_root(name, static=True)
    link = sdf.link(model, "link")
    image = media.sign(key, lines)
    sdf.shape(link, "post", sdf.box((0.05, 0.05, height + 0.2)), (-0.03, 0, (height + 0.2) / 2), (0.45, 0.33, 0.2))
    sdf.shape(link, "board", sdf.box((0.012, 0.62, 0.40)), (0, 0, height), WHITE)
    picture(link, media, "face", image, 0.6, 0.375, (0.0065, 0, height), emissive=True)
    sdf.write_model(models_dir, name, root, f"Field sign: {' / '.join(lines)}.")
    return name


def shrub(models_dir, variant):
    """Desert shrub (visual only: the rover drives through brush)."""
    name = f"urc_shrub_{variant}"
    root, model = sdf.model_root(name, static=True)
    link = sdf.link(model, "link")
    rng = np.random.default_rng(500 + variant)
    for k in range(int(rng.integers(6, 11))):
        r = rng.uniform(0.12, 0.25)
        x, y = rng.normal(0, 0.18, 2)
        g = rng.uniform(0.85, 1.15)
        sdf.visual(link, f"clump{k}", sdf.sphere(r), (x, y, r * 0.8 + rng.uniform(0, 0.15)),
                   (0.42 * g, 0.47 * g, 0.33 * g))
    sdf.write_model(models_dir, name, root, "Desert shrub.")
    return name


# --- Delivery objects (rule 1.c.iii: <= 5 kg, < 40 cm, grasp features <= 7.5 cm) ----

def toolbox(models_dir):
    """Toolbox with a hinged lid to open (rule 1.c.iii). Hinge along the back
    (-x) edge; the lid opens upwards."""
    name = "urc_toolbox"
    root, model = sdf.model_root(name)
    L, W, H, wall, lid_h = 0.38, 0.20, 0.13, 0.006, 0.035
    base = sdf.link(model, "base", None, 1.6, sdf.box_inertia(1.6, (L, W, H)), com=(0, 0, H / 3))
    sdf.shape(base, "floor", sdf.box((L, W, wall)), (0, 0, wall / 2), RED)
    for k, (size, xy) in enumerate((((wall, W, H), (L / 2 - wall / 2, 0)), ((wall, W, H), (-L / 2 + wall / 2, 0)),
                                    ((L, wall, H), (0, W / 2 - wall / 2)), ((L, wall, H), (0, -W / 2 + wall / 2)))):
        sdf.shape(base, f"wall{k}", sdf.box(size), (*xy, H / 2), RED)
    lid = sdf.link(model, "lid", (-L / 2, 0, H), 0.45, sdf.box_inertia(0.45, (L, W, lid_h)),
                   com=(L / 2, 0, lid_h / 2))
    sdf.shape(lid, "lid", sdf.box((L, W, lid_h)), (L / 2, 0, lid_h / 2), RED)
    handle_bar(lid, "handle", 0.16, 0.022, (L / 2, 0, lid_h), axis="y", posts=0.035)
    # Hinge axis -y: positive angle lifts the front edge.
    sdf.joint(model, "lid_hinge", "revolute", "base", "lid", (0, -1, 0), 0.0, 1.9, damping=0.05)
    monitor(model)
    sdf.write_model(models_dir, name, root, "Toolbox with a hinged lid.")
    return name


def wrench(models_dir):
    name = "urc_wrench"
    root, model = sdf.model_root(name)
    link = sdf.link(model, "link", None, 0.35, sdf.box_inertia(0.35, (0.25, 0.04, 0.01)), com=(0, 0, 0.005))
    sdf.shape(link, "handle", sdf.box((0.2, 0.024, 0.009)), (-0.02, 0, 0.0045), STEEL, metalness=0.8, roughness=0.3)
    sdf.shape(link, "head", sdf.box((0.05, 0.055, 0.012)), (0.1, 0, 0.006), STEEL, metalness=0.8, roughness=0.3)
    monitor(model)
    sdf.write_model(models_dir, name, root, "Wrench (hand tool to deliver).")
    return name


def _crate_like(models_dir, name, size, mass, color, handle_len, handle_dia, description, label=None, media=None):
    root, model = sdf.model_root(name)
    link = _box_link(model, "link", size, mass, color)
    handle_bar(link, "handle", handle_len, handle_dia, (0, 0, size[2]), axis="y", posts=0.045)
    if label is not None:
        picture(link, media, "label", label, size[1] * 0.8, size[2] * 0.6, (size[0] / 2 + 0.001, 0, size[2] / 2))
    monitor(model)
    sdf.write_model(models_dir, name, root, description)
    return name


def supply_crate(models_dir, media):
    label = media.sign("supplies", ["SUPPLIES"], size=(512, 256), bg=(230, 225, 200), border=(60, 70, 40))
    return _crate_like(models_dir, "urc_supply_crate", (0.34, 0.26, 0.22), 3.5, OLIVE, 0.16, 0.03,
                       "Supply container with a top handle.", label, media)


def instrument_case(models_dir):
    return _crate_like(models_dir, "urc_instrument_case", (0.36, 0.28, 0.14), 2.5, ORANGE, 0.15, 0.028,
                       "Orange instrument case (equipment to find in a large search area).")


def first_aid_kit(models_dir, media):
    cross = media.texture("red_cross", textures.label_image, "+", (256, 256), (245, 245, 245), (200, 20, 20))
    return _crate_like(models_dir, "urc_first_aid_kit", (0.26, 0.18, 0.10), 1.2, WHITE, 0.12, 0.025,
                       "First-aid kit with a top handle.", cross, media)


def water_jug(models_dir):
    name = "urc_water_jug"
    root, model = sdf.model_root(name)
    r, h, mass = 0.1, 0.3, 4.2
    link = sdf.link(model, "link", None, mass, sdf.cylinder_inertia(mass, r, h), com=(0, 0, h / 2))
    sdf.shape(link, "jug", sdf.cylinder(r, h), (0, 0, h / 2), (0.25, 0.5, 0.85), roughness=0.3)
    sdf.shape(link, "cap", sdf.cylinder(0.03, 0.03), (0, 0.04, h + 0.015), BLUE)
    handle_bar(link, "handle", 0.12, 0.03, (0, -0.02, h), axis="x", posts=0.04, color=BLUE)
    monitor(model)
    sdf.write_model(models_dir, name, root, "Water jug (4 L) with a handle.")
    return name


def spectrometer(models_dir):
    """Handheld instrument lost in the radio shadow."""
    name = "urc_spectrometer"
    root, model = sdf.model_root(name)
    link = _box_link(model, "link", (0.22, 0.08, 0.05), 0.6, YELLOW)
    sdf.shape(link, "grip", sdf.cylinder(0.02, 0.1), (-0.06, 0, 0.07), DARK)
    sdf.shape(link, "window", sdf.box((0.01, 0.05, 0.03)), (0.111, 0, 0.025), DARK, collide=False)
    monitor(model)
    sdf.write_model(models_dir, name, root, "Handheld spectrometer.")
    return name


# --- Equipment Servicing objects ------------------------------------------------------

SAMPLE_TUBE = {"radius": 0.008, "length": 0.125}
CACHE = {"outer": (0.12, 0.12, 0.15), "wall": 0.008, "lid": 0.022, "handle_length": 0.12, "handle_diameter": 0.03,
         "mass": 1.2}


def sample_tube(models_dir):
    """Test-tube sized sample tube (rule 1.d.ii) with a cap, standing."""
    name = "urc_sample_tube"
    root, model = sdf.model_root(name)
    r, length = SAMPLE_TUBE["radius"], SAMPLE_TUBE["length"]
    link = sdf.link(model, "link", None, 0.04, sdf.cylinder_inertia(0.04, r, length), com=(0, 0, length / 2))
    sdf.shape(link, "tube", sdf.cylinder(r, length - 0.02), (0, 0, (length - 0.02) / 2), (0.85, 0.8, 0.7),
              roughness=0.2)
    sdf.shape(link, "cap", sdf.cylinder(r + 0.0015, 0.02), (0, 0, length - 0.01), RED)
    monitor(model)
    sdf.write_model(models_dir, name, root, "Sample tube.")
    return name


def cache_container(models_dir):
    """Sample cache (rule 1.d.ii): open box with a hinged lid, a lock knob on
    the lid, and a 12 cm x 3 cm handle on the front."""
    name = "urc_cache_container"
    root, model = sdf.model_root(name)
    (L, W, H), t, lid_h = CACHE["outer"], CACHE["wall"], CACHE["lid"]
    body_mass = CACHE["mass"] - 0.25
    body = sdf.link(model, "body", None, body_mass, sdf.box_inertia(body_mass, (L, W, H)), com=(0, 0, H / 3))
    grey = (0.55, 0.57, 0.6)
    sdf.shape(body, "floor", sdf.box((L, W, t)), (0, 0, t / 2), grey)
    for k, (size, xy) in enumerate((((t, W, H), (L / 2 - t / 2, 0)), ((t, W, H), (-L / 2 + t / 2, 0)),
                                    ((L, t, H), (0, W / 2 - t / 2)), ((L, t, H), (0, -W / 2 + t / 2)))):
        sdf.shape(body, f"wall{k}", sdf.box(size), (*xy, H / 2), grey)
    # Handle across the front face (x+), horizontal.
    hl, hd = CACHE["handle_length"], CACHE["handle_diameter"]
    sdf.shape(body, "handle_bar", sdf.cylinder(hd / 2, hl), (L / 2 + 0.035, 0, H * 0.55, math.pi / 2, 0, 0), DARK)
    for k, y in enumerate((hl / 2 - 0.008, -hl / 2 + 0.008)):
        sdf.shape(body, f"handle_post{k}", sdf.box((0.035, 0.012, 0.012)), (L / 2 + 0.0175, y, H * 0.55), DARK)
    lid = sdf.link(model, "lid", (-L / 2, 0, H), 0.2, sdf.box_inertia(0.2, (L, W, lid_h)), com=(L / 2, 0, lid_h / 2))
    sdf.shape(lid, "lid", sdf.box((L, W, lid_h)), (L / 2, 0, lid_h / 2), (0.3, 0.32, 0.36))
    sdf.joint(model, "lid_hinge", "revolute", "body", "lid", (0, -1, 0), 0.0, 1.9, damping=0.02)
    lock = sdf.link(model, "lock", (-L / 2 + L * 0.7, 0, H + lid_h), 0.05, sdf.cylinder_inertia(0.05, 0.02, 0.02))
    sdf.shape(lock, "knob", sdf.cylinder(0.018, 0.012), (0, 0, 0.006), YELLOW)
    sdf.shape(lock, "tab", sdf.box((0.034, 0.008, 0.016)), (0, 0, 0.02), YELLOW)
    sdf.joint(model, "lock_joint", "revolute", "lid", "lock", (0, 0, 1), 0.0, math.pi / 2, damping=0.01, friction=0.05)
    monitor(model)
    sdf.write_model(models_dir, name, root, "Sample cache container with lid, lock and handle.")
    return name


def sample_stand(models_dir):
    """Low table holding the sample tube (in a rack) and the cache."""
    name = "urc_sample_stand"
    root, model = sdf.model_root(name, static=True)
    link = sdf.link(model, "link")
    top = 0.5
    sdf.shape(link, "top", sdf.box((0.6, 0.45, 0.03)), (0, 0, top - 0.015), (0.5, 0.35, 0.2))
    for k, (x, y) in enumerate(((0.27, 0.2), (0.27, -0.2), (-0.27, 0.2), (-0.27, -0.2))):
        sdf.shape(link, f"leg{k}", sdf.box((0.04, 0.04, top - 0.03)), (x, y, (top - 0.03) / 2), DARK)
    # Rack for the tube at (0.15, -0.12): four blocks around a 20 mm hole.
    for k, (dx, dy, sx, sy) in enumerate(((0.02, 0, 0.02, 0.06), (-0.02, 0, 0.02, 0.06), (0, 0.02, 0.02, 0.02),
                                          (0, -0.02, 0.02, 0.02))):
        sdf.shape(link, f"rack{k}", sdf.box((sx, sy, 0.05)), (0.15 + dx, -0.12 + dy, top + 0.025), (0.2, 0.2, 0.22))
    sdf.write_model(models_dir, name, root, "Sample stand with a tube rack.")
    return name


SAMPLE_STAND_TUBE = (0.15, -0.12, 0.5)  # where the tube stands, in the stand frame
SAMPLE_STAND_CACHE = (-0.1, 0.05, 0.5)

HOSE_SEGMENTS = 8
HOSE_SEGMENT = 0.3
HOSE_DIAMETER = 0.05
COUPLER = {"radius": 0.036, "inner_radius": 0.0275, "length": 0.09}


def fuel_tank(models_dir):
    """Fuel tank anchored to the ground with a hose ending in a 1.5" cam-lock
    coupler (rule 1.d.ii). The hose lies along +x from the outlet."""
    name = "urc_fuel_tank"
    root, model = sdf.model_root(name)
    tank = sdf.link(model, "tank", None, 30.0, sdf.cylinder_inertia(30.0, 0.25, 0.9), com=(0, 0, 0.45))
    sdf.shape(tank, "tank", sdf.cylinder(0.25, 0.9), (0, 0, 0.45), RED)
    sdf.shape(tank, "outlet", sdf.cylinder(0.03, 0.1), (0.29, 0, 0.05, 0, math.pi / 2, 0), STEEL)
    sdf.joint(model, "anchor", "fixed", "world", "tank")
    previous = "tank"
    x0 = 0.34
    seg_mass = 0.18
    for k in range(HOSE_SEGMENTS):
        child = f"hose_{k}"
        link = sdf.link(model, child, (x0 + k * HOSE_SEGMENT, 0, HOSE_DIAMETER / 2), seg_mass,
                        sdf.cylinder_inertia(seg_mass, HOSE_DIAMETER / 2, HOSE_SEGMENT), com=(HOSE_SEGMENT / 2, 0, 0))
        sdf.shape(link, "hose", sdf.cylinder(HOSE_DIAMETER / 2, HOSE_SEGMENT - 0.004),
                  (HOSE_SEGMENT / 2, 0, 0, 0, math.pi / 2, 0), DARK, mu=0.8)
        j = sdf.joint(model, f"hose_joint_{k}", "universal", previous, child, (0, 0, 1))
        axis2 = sdf.sub(j, "axis2")
        sdf.sub(axis2, "xyz", (0, 1, 0))
        for ax in (j.find("axis"), axis2):
            sdf.sub(sdf.sub(ax, "dynamics"), "damping", 0.3)
        previous = child
    coupler = sdf.link(model, "coupler", (x0 + HOSE_SEGMENTS * HOSE_SEGMENT, 0, HOSE_DIAMETER / 2), 0.45,
                       sdf.cylinder_inertia(0.45, COUPLER["radius"], COUPLER["length"]),
                       com=(COUPLER["length"] / 2, 0, 0))
    sdf.visual(coupler, "body", sdf.cylinder(COUPLER["radius"], COUPLER["length"]),
               (COUPLER["length"] / 2, 0, 0, 0, math.pi / 2, 0), (0.75, 0.6, 0.25), metalness=0.6, roughness=0.4)
    # Hollow (an octagonal tube) so it can be pushed onto the lander's inlet.
    apothem, wall = COUPLER["inner_radius"] + 0.003, 0.006
    side = 2 * (apothem + wall / 2) * math.tan(math.pi / 8)
    for k in range(8):
        a = k * math.pi / 4
        sdf.collision(coupler, f"wall{k}", sdf.box((COUPLER["length"], side, wall)),
                      (COUPLER["length"] / 2, -math.sin(a) * apothem, math.cos(a) * apothem, a, 0, 0))
    for k, y in enumerate((1, -1)):
        sdf.shape(coupler, f"cam_arm{k}", sdf.box((0.07, 0.01, 0.012)), (0.05, y * (COUPLER["radius"] + 0.006), 0),
                  STEEL)
    sdf.joint(model, "coupler_joint", "fixed", previous, "coupler")
    monitor(model)
    sdf.write_model(models_dir, name, root, "Fuel tank with hose and cam-lock coupler.")
    return name
