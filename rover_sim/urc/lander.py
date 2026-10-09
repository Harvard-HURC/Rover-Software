"""Equipment Servicing mock lander (rule 1.d) and its tubular key.

Lander frame: origin on the ground at the middle of the front face, x out of
the face (towards the rover), y along the face (left as the rover sees it is
-y), z up. Everything to service is on the face between the ground and 1.5 m.

The base is fixed to the world; moving parts are joints:

| part | joint(s) | type | judged as |
|---|---|---|---|
| keyboard (87 keys) | key_<id> | prismatic, spring, 4 mm | presses -> typed text |
| drawer | drawer | prismatic, 0-0.35 m out | open, then closed with the cache in the well |
| hinged panel | door | revolute, 0-1.9 rad | opened |
| latch | latch | revolute, 0-pi/2 | undone (> 1.2 rad); blocks the door at 0 |
| locks A, B | lock_a, lock_b | revolute, 0-pi/2 | key moved A -> B and B turned |
| valve | valve | revolute, 0-pi/2 | turned |
| buttons | button_<n> | prismatic, spring, 8 mm | pressed |
| switches | switch_<n> | revolute, 0-0.9 rad | flipped (> 0.6) |
| knobs | knob_<n> | revolute, +-2.6 rad | turned (|q| > 1.4) |

The model self-collides so the latch can hold the door; the geometry keeps
every other pair of parts apart. JointMonitor publishes the joint states
and key/button presses.
"""
import math

from . import props, rules, sdf, textures

NAME = "urc_lander"
KEY_NAME = "urc_tubular_key"
WIDTH, HEIGHT, DEPTH = 2.4, 1.6, 1.2
SLAB = 0.5  # depth of the front slab that holds the drawer

PITCH = 0.01905  # key pitch [m], 1 u
KEY_GAP = 0.0012
KEY_TRAVEL = 0.004
KEYBOARD_CENTER = (-0.75, 0.85)  # (y, z)
KEYBOARD_SIZE = (0.354, 0.123)  # Redragon K552 (y, z)
KEYBOARD_DEPTH = 0.03  # case, out of the face
DISPLAY_CENTER = (-0.75, 1.08)

DRAWER_CENTER = (-0.22, 0.42)
DRAWER_OPENING = (0.32, 0.24)
DRAWER_DEPTH = 0.45
DRAWER_TRAVEL = 0.35
WELL_X = -0.2  # cache well centre in the drawer frame
WELL_INNER = 0.135

DOOR_HINGE_Y, DOOR_Z, DOOR_SIZE = 0.08, 1.15, (0.36, 0.30)
LATCH_Y = DOOR_HINGE_Y + DOOR_SIZE[0] + 0.035
LOCK_A, LOCK_B = (0.68, 1.30), (0.98, 1.30)
LOCK_DEPTH = 0.025
BORE = 0.0135  # square bore of a lock plug
BUTTONS = [(0.64, 1.02, (0.8, 0.1, 0.08)), (0.78, 1.02, (0.1, 0.65, 0.15)), (0.92, 1.02, (0.9, 0.75, 0.05))]
SWITCHES = [(0.64, 0.88), (0.78, 0.88), (0.92, 0.88)]
KNOBS = [(1.10, 1.02), (1.10, 0.86)]
VALVE = (0.75, 0.62)
INLET = (1.0, 0.42)
INLET_RADIUS = 0.026
INLET_LENGTH = 0.06

SWITCH_FLIPPED = 0.6
KNOB_TURNED = 1.4
LATCH_OPEN = 1.2

PANEL_GREY = (0.32, 0.33, 0.35)
DARK = props.DARK
DARK_FACE = (0.08, 0.08, 0.09)
FOIL = (0.83, 0.68, 0.32)


def _keyboard_layout():
    """[(joint id, legend, typed character or None, x [u], row [u], width [u])],
    x and row measured to the key's top-left corner from the keyboard's."""
    keys = []

    def row(y, items, x=0.0):
        for item in items:
            if item is None:
                x += 0.5
                continue
            ident, legend, char, w = item
            keys.append((ident, legend, char, x, y, w))
            x += w

    def k(ident, legend=None, char=None, w=1.0):
        return (ident, legend or ident.upper(), char, w)

    letters = lambda s: [k(c, c.upper(), c) for c in s]  # noqa: E731
    fkeys = lambda a, b: [k(f"f{n}", f"F{n}") for n in range(a, b)]  # noqa: E731
    row(0, [k("esc", "Esc"), None, None] + fkeys(1, 5) + [None] + fkeys(5, 9) + [None] + fkeys(9, 13))
    row(0, [k("prtsc", "PrtSc"), k("scrlk", "ScrLk"), k("pause", "Pause")], x=15.25)
    row(1.5, [k("grave", "`", "`")] + [k(c, c, c) for c in "1234567890"] + [k("minus", "-", "-"), k("equal", "=", "=")]
        + [k("backspace", "Bksp", "\b", 2.0)])
    row(1.5, [k("insert", "Ins"), k("home", "Home"), k("pgup", "PgUp")], x=15.25)
    row(2.5, [k("tab", "Tab", None, 1.5)] + letters("qwertyuiop") + [k("lbracket", "[", "["), k("rbracket", "]", "]"),
                                                                        k("backslash", "\\", "\\", 1.5)])
    row(2.5, [k("delete", "Del", "\x7f"), k("end", "End"), k("pgdn", "PgDn")], x=15.25)
    row(3.5, [k("caps", "Caps", None, 1.75)] + letters("asdfghjkl") + [k("semicolon", ";", ";"),
                                                                        k("quote", "'", "'"), k("enter", "Enter", "\n", 2.25)])
    row(4.5, [k("lshift", "Shift", None, 2.25)] + letters("zxcvbnm") + [k("comma", ",", ","), k("period", ".", "."),
                                                                          k("slash", "/", "/"), k("rshift", "Shift", None, 2.75)])
    row(4.5, [k("up", "^")], x=16.25)
    row(5.5, [k("lctrl", "Ctrl", None, 1.25), k("lwin", "Win", None, 1.25), k("lalt", "Alt", None, 1.25),
              k("space", "", " ", 6.25), k("ralt", "Alt", None, 1.25), k("fn", "Fn", None, 1.25),
              k("menu", "Menu", None, 1.25), k("rctrl", "Ctrl", None, 1.25)])
    row(5.5, [k("left", "<"), k("down", "v"), k("right", ">")], x=15.25)
    return keys


KEYS = _keyboard_layout()
KEYMAP = {f"key_{ident}": char for ident, _, char, *_ in KEYS}  # joint -> typed character (None: no text)
assert len(KEYS) == rules.KEYBOARD_KEYS


def key_center(x, row, width):
    """Key centre (y, z) on the face from its layout position."""
    total_w, total_h = 18.25, 6.5
    y = KEYBOARD_CENTER[0] + (x + width / 2 - total_w / 2) * PITCH
    z = KEYBOARD_CENTER[1] + (total_h / 2 - (row + 0.5)) * PITCH
    return y, z


def to_world(lander_pose, xyz):
    """A point in the lander frame -> world, for a lander at (x, y, z, yaw)."""
    x0, y0, z0, yaw = lander_pose
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, z = xyz
    return (x0 + c * x - s * y, y0 + s * x + c * y, z0 + z)


def lock_point(lock, depth=0.0):
    """Mouth of a lock bore (depth 0) in the lander frame."""
    y, z = {"a": LOCK_A, "b": LOCK_B}[lock]
    return (LOCK_DEPTH - depth, y, z)


KEY_TIP_DEPTH = 0.02  # how far the key barrel is in its lock
KEY_START = (*lock_point("a", KEY_TIP_DEPTH), 0.0, 0.0, 0.0)  # key model pose in the lander frame, inserted in A
INLET_TIP = (INLET_LENGTH, *INLET)
DRAWER_WELL_CLOSED = (WELL_X, DRAWER_CENTER[0], DRAWER_CENTER[1] - 0.105)  # well floor centre, drawer closed


def _box_between(link, name, lo, hi, color, collide=True):
    size = tuple(h - l for l, h in zip(lo, hi))
    center = tuple((l + h) / 2 for l, h in zip(lo, hi))
    sdf.shape(link, name, sdf.box(size), center, color, collide=collide)


def _body(model, media):
    base = sdf.link(model, "base", None, 400.0, sdf.box_inertia(400.0, (DEPTH, WIDTH, HEIGHT)),
                    com=(-DEPTH / 2, 0, HEIGHT / 2))
    w, h = WIDTH / 2, HEIGHT
    dy, dz = DRAWER_CENTER
    oy, oz = DRAWER_OPENING[0] / 2, DRAWER_OPENING[1] / 2
    _box_between(base, "back", (-DEPTH, -w, 0), (-SLAB, w, h), (0.9, 0.9, 0.88))
    # The front slab, around the drawer opening.
    _box_between(base, "slab_left", (-SLAB, -w, 0), (0, dy - oy, h), PANEL_GREY)
    _box_between(base, "slab_right", (-SLAB, dy + oy, 0), (0, w, h), PANEL_GREY)
    _box_between(base, "slab_top", (-SLAB, dy - oy, dz + oz), (0, dy + oy, h), PANEL_GREY)
    _box_between(base, "slab_bottom", (-SLAB, dy - oy, 0), (0, dy + oy, dz - oz), PANEL_GREY)
    _box_between(base, "cavity", (-SLAB - 0.001, dy - oy, dz - oz), (-SLAB, dy + oy, dz + oz), DARK_FACE, False)
    # Lander dressing above the work area (nothing to service up there).
    sdf.shape(base, "deck", sdf.cylinder(1.0, 0.25), (-DEPTH / 2, 0, h + 0.125), FOIL, metalness=0.7,
              roughness=0.35)
    sdf.shape(base, "dish", sdf.cylinder(0.35, 0.04), (-DEPTH / 2, 0.5, h + 0.6, 0.5, 0, 0), (0.9, 0.9, 0.9),
              collide=False)
    sdf.shape(base, "mast", sdf.cylinder(0.03, 0.4), (-DEPTH / 2, 0.5, h + 0.42), PANEL_GREY, collide=False)
    for k, (x, y) in enumerate(((-0.05, -w - 0.15), (-0.05, w + 0.15), (-DEPTH + 0.05, -w - 0.15),
                                (-DEPTH + 0.05, w + 0.15))):
        sdf.shape(base, f"leg{k}", sdf.cylinder(0.05, 1.0), (x, y, 0.6, 0.3 if y > 0 else -0.3, 0, 0), FOIL,
                  collide=False, metalness=0.7, roughness=0.35)
        sdf.shape(base, f"foot{k}", sdf.cylinder(0.15, 0.04), (x, y + (0.15 if y > 0 else -0.15), 0.02), FOIL)
    label = media.sign("lander", ["URC LANDER"], size=(1024, 192), bg=(40, 42, 46), fg=(240, 240, 240),
                       border=(240, 120, 20))
    props.picture(base, media, "nameplate", label, 0.8, 0.15, (0.001, 0.4, 1.47))
    return base


def _keyboard(model, base, media):
    cy, cz = KEYBOARD_CENTER
    W, H = KEYBOARD_SIZE
    # Black mounting plate with the four alignment tags (rule 1.d.ii figure).
    plate = (W + 0.09, H + 0.07)
    sdf.shape(base, "kb_plate", sdf.box((0.004, *plate)), (0.002, cy, cz), DARK_FACE)
    sdf.shape(base, "kb_case", sdf.box((KEYBOARD_DEPTH, W, H)), (KEYBOARD_DEPTH / 2, cy, cz), (0.15, 0.15, 0.16))
    t = rules.KEYBOARD_TAG_SIZE
    for corner, tag in rules.KEYBOARD_TAG_IDS.items():
        sy = -1 if "left" in corner else 1  # the rover's left is -y
        sz = 1 if "top" in corner else -1
        props.picture(base, media, f"kb_tag_{corner}", media.aruco(tag), t, t,
                      (0.0045, cy + sy * (W / 2 + 0.022), cz + sz * (H / 2 + 0.015)), emissive=True)
    # 4.3" e-paper display above the keyboard.
    dy, dz = DISPLAY_CENTER
    sdf.shape(base, "display_bezel", sdf.box((0.012, 0.118, 0.075)), (0.006, dy, dz), (0.9, 0.9, 0.9))
    screen = media.texture("epaper", textures.label_image, "_", (400, 300), (215, 215, 205), (40, 40, 40))
    props.picture(base, media, "display", screen, 0.088, 0.066, (0.0125, dy, dz))
    for ident, legend, _, x, row, width in KEYS:
        y, z = key_center(x, row, width)
        name = f"key_{ident}"
        cap = (0.012, width * PITCH - KEY_GAP, PITCH - KEY_GAP)
        link = sdf.link(model, f"{name}_link", (KEYBOARD_DEPTH, y, z), 0.004, sdf.box_inertia(0.004, cap),
                        com=(cap[0] / 2, 0, 0))
        sdf.shape(link, "cap", sdf.box(cap), (cap[0] / 2, 0, 0), (0.08, 0.08, 0.09))
        image = media.texture(f"key_{ident}", textures.label_image, legend or " ", (int(64 * width), 64),
                              (22, 22, 24), (230, 40, 40))
        props.picture(link, media, "legend", image, cap[1] - 0.002, cap[2] - 0.002, (cap[0] + 0.0004, 0, 0))
        sdf.joint(model, name, "prismatic", "base", f"{name}_link", (-1, 0, 0), 0.0, KEY_TRAVEL, effort=50,
                  damping=1.2, stiffness=300.0, reference=0.0)


def _drawer(model, base):
    dy, dz = DRAWER_CENTER
    link = sdf.link(model, "drawer_box", (0, dy, dz), 2.0, sdf.box_inertia(2.0, (DRAWER_DEPTH, 0.3, 0.22)),
                    com=(-DRAWER_DEPTH / 2, 0, -0.05))
    ow, oh = DRAWER_OPENING
    sdf.shape(link, "front", sdf.box((0.02, ow + 0.03, oh + 0.03)), (0.01, 0, 0), (0.5, 0.52, 0.55))
    inner_w = ow - 0.02
    floor_z = -oh / 2 + 0.01
    _box_between(link, "floor", (-DRAWER_DEPTH, -inner_w / 2, floor_z - 0.01), (0, inner_w / 2, floor_z), DARK)
    for k, (y0, y1) in enumerate(((-inner_w / 2, -inner_w / 2 + 0.005), (inner_w / 2 - 0.005, inner_w / 2))):
        _box_between(link, f"side{k}", (-DRAWER_DEPTH, y0, floor_z), (0, y1, oh / 2 - 0.02), DARK)
    _box_between(link, "back", (-DRAWER_DEPTH, -inner_w / 2, floor_z), (-DRAWER_DEPTH + 0.005, inner_w / 2,
                                                                         oh / 2 - 0.02), DARK)
    # Tight-fitting well for the cache (rule 1.d.ii), 6 cm deep.
    half, t, wall_h = WELL_INNER / 2, 0.006, 0.06
    for k, (lo, hi) in enumerate((((WELL_X - half - t, -half - t), (WELL_X - half, half + t)),
                                  ((WELL_X + half, -half - t), (WELL_X + half + t, half + t)),
                                  ((WELL_X - half, -half - t), (WELL_X + half, -half)),
                                  ((WELL_X - half, half), (WELL_X + half, half + t)))):
        _box_between(link, f"well{k}", (lo[0], lo[1], floor_z), (hi[0], hi[1], floor_z + wall_h), props.YELLOW)
    # Pull handle sticking out of the front.
    for k, y in enumerate((-0.06, 0.06)):
        sdf.shape(link, f"handle_post{k}", sdf.cylinder(0.006, 0.035), (0.0375, y, 0, 0, math.pi / 2, 0), DARK)
    sdf.shape(link, "handle_bar", sdf.cylinder(0.011, 0.14), (0.055, 0, 0, math.pi / 2, 0, 0), DARK)
    sdf.joint(model, "drawer", "prismatic", "base", "drawer_box", (1, 0, 0), 0.0, DRAWER_TRAVEL, damping=3.0,
              friction=1.0)


def _door_and_latch(model, base):
    w, h = DOOR_SIZE
    sdf.shape(base, "compartment", sdf.box((0.001, w - 0.02, h - 0.02)),
              (0.0005, DOOR_HINGE_Y + w / 2, DOOR_Z), DARK_FACE, collide=False)
    door = sdf.link(model, "door_link", (0, DOOR_HINGE_Y, DOOR_Z), 0.8, sdf.box_inertia(0.8, (0.02, w, h)),
                    com=(0.011, w / 2, 0))
    sdf.shape(door, "plate", sdf.box((0.02, w, h)), (0.011, w / 2, 0), (0.55, 0.57, 0.6))
    sdf.shape(door, "pull", sdf.box((0.025, 0.02, 0.08)), (0.0335, w - 0.06, 0), DARK)
    sdf.joint(model, "door", "revolute", "base", "door_link", (0, 0, -1), 0.0, 1.9, damping=0.3)
    latch = sdf.link(model, "latch_link", (0, LATCH_Y, DOOR_Z), 0.05, sdf.box_inertia(0.05, (0.03, 0.08, 0.02)),
                     com=(0.02, -0.03, 0))
    sdf.shape(latch, "hub", sdf.cylinder(0.012, 0.03), (0.019, 0, 0, 0, math.pi / 2, 0), props.RED)
    # At 0 the arm lies over the door's free edge, in front of it.
    sdf.shape(latch, "arm", sdf.box((0.012, 0.075, 0.02)), (0.029, -0.0375, 0), props.RED)
    sdf.joint(model, "latch", "revolute", "base", "latch_link", (1, 0, 0), 0.0, math.pi / 2, damping=0.02, friction=0.05)


def _locks(model, base, media):
    for lock, (y, z) in (("a", LOCK_A), ("b", LOCK_B)):
        sdf.visual(base, f"lock_{lock}_housing", sdf.cylinder(0.022, LOCK_DEPTH),
                   (LOCK_DEPTH / 2 - 0.001, y, z, 0, math.pi / 2, 0), props.STEEL, metalness=0.8, roughness=0.3)
        for corner, tag in rules.KEYBOARD_TAG_IDS.items():
            sy = -1 if "left" in corner else 1
            sz = 1 if "top" in corner else -1
            props.picture(base, media, f"lock_{lock}_tag_{corner}", media.aruco(tag), rules.LOCK_TAG_SIZE,
                          rules.LOCK_TAG_SIZE, (0.0005, y + sy * 0.032, z + sz * 0.032), emissive=True)
        plug = sdf.link(model, f"lock_{lock}_link", (0, y, z), 0.05, sdf.box_inertia(0.05, (LOCK_DEPTH, 0.024, 0.024)),
                        com=(LOCK_DEPTH / 2, 0, 0))
        # A square bore (so a square key barrel can turn the plug) through the plug.
        half, t = BORE / 2, 0.005
        for k, (lo, hi) in enumerate((((0, -half - t, -half - t), (LOCK_DEPTH, half + t, -half)),
                                      ((0, -half - t, half), (LOCK_DEPTH, half + t, half + t)),
                                      ((0, -half - t, -half), (LOCK_DEPTH, -half, half)),
                                      ((0, half, -half), (LOCK_DEPTH, half + t, half)))):
            _box_between(plug, f"bore{k}", lo, hi, (0.7, 0.7, 0.72))
        sdf.shape(plug, "face", sdf.box((0.001, 0.03, 0.03)), (LOCK_DEPTH + 0.0005, 0, 0), (0.5, 0.5, 0.52),
                  collide=False)
        sdf.joint(model, f"lock_{lock}", "revolute", "base", f"lock_{lock}_link", (1, 0, 0), 0.0, math.pi / 2,
                  damping=0.01, friction=0.02)


def _controls(model, base):
    # Quarter-turn valve: pipe stub, body, lever turning about x.
    vy, vz = VALVE
    sdf.shape(base, "pipe", sdf.cylinder(0.02, 0.3), (0.03, vy, vz, math.pi / 2, 0, 0), props.STEEL)
    sdf.shape(base, "valve_body", sdf.box((0.05, 0.06, 0.06)), (0.03, vy, vz), (0.75, 0.6, 0.25))
    lever = sdf.link(model, "valve_link", (0.055, vy, vz), 0.2, sdf.box_inertia(0.2, (0.02, 0.16, 0.02)),
                     com=(0.012, 0.07, 0))
    sdf.shape(lever, "stem", sdf.cylinder(0.008, 0.012), (0.006, 0, 0, 0, math.pi / 2, 0), props.STEEL)
    sdf.shape(lever, "lever", sdf.box((0.01, 0.16, 0.022)), (0.017, 0.07, 0), props.RED)
    sdf.joint(model, "valve", "revolute", "base", "valve_link", (1, 0, 0), 0.0, rules.VALVE_TURN, damping=0.2,
              friction=0.3)
    for n, (y, z, color) in enumerate(BUTTONS):
        sdf.shape(base, f"button_{n}_ring", sdf.cylinder(0.026, 0.01), (0.005, y, z, 0, math.pi / 2, 0), DARK)
        b = sdf.link(model, f"button_{n}_link", (0.01, y, z), 0.02, sdf.cylinder_inertia(0.02, 0.018, 0.02),
                     com=(0.01, 0, 0))
        sdf.shape(b, "cap", sdf.cylinder(0.018, 0.02), (0.01, 0, 0, 0, math.pi / 2, 0), color)
        sdf.joint(model, f"button_{n}", "prismatic", "base", f"button_{n}_link", (-1, 0, 0), 0.0, 0.008, damping=1.0,
                  stiffness=150.0, reference=0.0)
    tilt = 0.45  # a switch at 0 points out and down; flipped it points out and up
    for n, (y, z) in enumerate(SWITCHES):
        sdf.shape(base, f"switch_{n}_plate", sdf.box((0.006, 0.03, 0.05)), (0.003, y, z), props.STEEL)
        s = sdf.link(model, f"switch_{n}_link", (0.012, y, z), 0.01, sdf.cylinder_inertia(0.01, 0.004, 0.03),
                     com=(0.015, 0, 0))
        sdf.shape(s, "bat", sdf.cylinder(0.004, 0.03),
                  (0.015 * math.cos(tilt), 0, -0.015 * math.sin(tilt), 0, math.pi / 2 + tilt, 0), props.STEEL)
        sdf.joint(model, f"switch_{n}", "revolute", "base", f"switch_{n}_link", (0, -1, 0), 0.0, 2 * tilt, damping=0.01,
                  friction=0.02)
    for n, (y, z) in enumerate(KNOBS):
        sdf.shape(base, f"knob_{n}_dial", sdf.cylinder(0.035, 0.002), (0.001, y, z, 0, math.pi / 2, 0),
                  (0.85, 0.85, 0.85), collide=False)
        knob = sdf.link(model, f"knob_{n}_link", (0.002, y, z), 0.03, sdf.cylinder_inertia(0.03, 0.025, 0.025),
                        com=(0.0125, 0, 0))
        sdf.shape(knob, "body", sdf.cylinder(0.025, 0.025), (0.0125, 0, 0, 0, math.pi / 2, 0), DARK)
        sdf.shape(knob, "pointer", sdf.box((0.006, 0.006, 0.022)), (0.026, 0, 0.012), (0.95, 0.95, 0.95))
        sdf.joint(model, f"knob_{n}", "revolute", "base", f"knob_{n}_link", (1, 0, 0), -2.6, 2.6, damping=0.02,
                  friction=0.05)
    # Male cam-lock adapter (push-on hose connection, rule 1.d.ii).
    iy, iz = INLET
    sdf.shape(base, "inlet", sdf.cylinder(INLET_RADIUS, INLET_LENGTH), (INLET_LENGTH / 2, iy, iz, 0, math.pi / 2, 0),
              (0.75, 0.6, 0.25), metalness=0.6, roughness=0.4)
    sdf.shape(base, "inlet_flange", sdf.cylinder(0.045, 0.01), (0.005, iy, iz, 0, math.pi / 2, 0), props.STEEL)


def lander(models_dir, media):
    root, model = sdf.model_root(NAME)
    sdf.sub(model, "self_collide", True)
    base = _body(model, media)
    sdf.joint(model, "anchor", "fixed", "world", "base")
    _keyboard(model, base, media)
    _drawer(model, base)
    _door_and_latch(model, base)
    _locks(model, base, media)
    _controls(model, base)
    sdf.plugin(model, "JointMonitor", "rover_sim::JointMonitor", update_rate=50,
               press_prefix=["key_", "button_"], press_threshold=0.002)
    for collision in model.iter("collision"):  # the base is fixed: no part reaches the ground (sdf.ABOVE_GROUND)
        sdf.collide_bitmask(collision, sdf.ABOVE_GROUND)
    sdf.write_model(models_dir, NAME, root, "URC Equipment Servicing mock lander.")
    return NAME


def tubular_key(models_dir, media):
    """The key for the lock task: a square barrel (so it can turn a plug) and
    a head with 1 cm tags 5 and 6 on its two faces (rule 1.d.ii). Origin at the
    barrel tip; the barrel runs along +x into the head."""
    root, model = sdf.model_root(KEY_NAME)
    barrel, head = 0.022, (0.028, 0.003, 0.024)
    link = sdf.link(model, "link", None, 0.015, sdf.box_inertia(0.015, (0.05, 0.012, 0.024)), com=(0.03, 0, 0))
    sdf.shape(link, "barrel", sdf.box((barrel, 0.012, 0.012)), (barrel / 2, 0, 0), props.STEEL, metalness=0.8,
              roughness=0.3)
    sdf.shape(link, "head", sdf.box(head), (barrel + head[0] / 2, 0, 0), props.STEEL, metalness=0.8, roughness=0.3)
    size = rules.LOCK_TAG_SIZE
    for side, tag in zip((1, -1), rules.KEY_TAG_IDS):
        props.picture(link, media, f"tag_{tag}", media.aruco(tag), size, size,
                      (barrel + head[0] / 2, side * (head[1] / 2 + 0.0004), 0, 0, 0, side * math.pi / 2), emissive=True)
    props.monitor(model)
    sdf.write_model(models_dir, KEY_NAME, root, "Tubular key with ArUco tags 5 and 6.")
    return KEY_NAME
