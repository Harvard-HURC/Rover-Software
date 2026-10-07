"""Small helpers for writing SDF 1.11 with xml.etree: every generated model
(the rover and its cameras in gen_model.py, the URC worlds and models) is
written with them.

Geometry builders (box, cylinder, ...) return a function that fills a
<geometry> element, so one shape can be used for a collision and a visual.
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

SDF_VERSION = "1.11"
WHITE = (1.0, 1.0, 1.0, 1.0)


def fmt(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (tuple, list)):
        return " ".join(fmt(v) for v in value)
    if isinstance(value, str):
        return value
    return f"{value:.9g}"


def sub(parent, tag, text=None, **attrib):
    element = ET.SubElement(parent, tag, {k: str(v) for k, v in attrib.items()})
    if text is not None:
        element.text = fmt(text)
    return element


def pose(parent, xyzrpy, relative_to=None):
    xyzrpy = tuple(xyzrpy) + (0.0,) * (6 - len(xyzrpy))
    attrib = {"relative_to": relative_to} if relative_to else {}
    return sub(parent, "pose", xyzrpy, **attrib)


# --- Inertia -------------------------------------------------------------------

def box_inertia(mass, size):
    x, y, z = size
    return (mass * (y * y + z * z) / 12, mass * (x * x + z * z) / 12, mass * (x * x + y * y) / 12)


def cylinder_inertia(mass, radius, length):
    """Solid cylinder whose axis is z."""
    across = mass * (3 * radius * radius + length * length) / 12
    return (across, across, mass * radius * radius / 2)


def inertial(link, mass, moments, xyz=(0, 0, 0)):
    element = sub(link, "inertial")
    pose(element, xyz)
    sub(element, "mass", mass)
    inertia = sub(element, "inertia")
    for key, value in zip(("ixx", "iyy", "izz"), moments):
        sub(inertia, key, max(value, 1e-6))
    for key in ("ixy", "ixz", "iyz"):
        sub(inertia, key, 0.0)
    return element


# --- Geometry ------------------------------------------------------------------

def box(size):
    return lambda g: sub(sub(g, "box"), "size", size)


def cylinder(radius, length):
    def build(g):
        c = sub(g, "cylinder")
        sub(c, "radius", radius)
        sub(c, "length", length)
    return build


def sphere(radius):
    return lambda g: sub(sub(g, "sphere"), "radius", radius)


def plane(size, normal=(0, 0, 1)):
    def build(g):
        p = sub(g, "plane")
        sub(p, "normal", normal)
        sub(p, "size", size)
    return build


def mesh(uri, scale=None):
    def build(g):
        m = sub(g, "mesh")
        sub(m, "uri", uri)
        if scale is not None:
            sub(m, "scale", scale if isinstance(scale, (tuple, list)) else (scale,) * 3)
    return build


# --- Links, shapes, joints -----------------------------------------------------

def material(element, color=WHITE, albedo=None, normal=None, emissive=None, roughness=0.9, metalness=0.0,
             emissive_map=None, plain=False):
    """A PBR colour, or a PBR texture (albedo_map) when albedo is a URI;
    plain: the classic ambient, diffuse and emissive only (the rover's look)."""
    m = sub(element, "material")
    if albedo is not None:
        color = WHITE
    rgba = tuple(color) + (1.0,) * (4 - len(color))
    sub(m, "ambient", rgba)
    sub(m, "diffuse", rgba)
    if not plain:
        sub(m, "specular", (0.1, 0.1, 0.1, 1))
    if emissive is not None:
        sub(m, "emissive", tuple(emissive) + (1.0,) * (4 - len(emissive)))
    if plain:
        return m
    metal = sub(sub(m, "pbr"), "metal")
    if albedo is not None:
        sub(metal, "albedo_map", albedo)
    if normal is not None:
        sub(metal, "normal_map", normal)
    if emissive_map is not None:
        sub(metal, "emissive_map", emissive_map)
    sub(metal, "roughness", roughness)
    sub(metal, "metalness", metalness)
    return m


def link(model, name, xyzrpy=None, mass=None, moments=None, com=(0, 0, 0), gravity=True, relative_to=None):
    element = sub(model, "link", name=name)
    if xyzrpy is not None:
        pose(element, xyzrpy, relative_to)
    if mass is not None:
        inertial(element, mass, moments, com)
    if not gravity:
        sub(element, "gravity", False)
    return element


def collision(link_element, name, geometry, xyzrpy=(0, 0, 0), mu=None, mu2=None, fdir1=None):
    """mu along fdir1 (in the collision frame) and mu2 across it, default mu."""
    element = sub(link_element, "collision", name=f"{name}_collision")
    pose(element, xyzrpy)
    geometry(sub(element, "geometry"))
    if mu is not None:
        ode = sub(sub(sub(element, "surface"), "friction"), "ode")
        sub(ode, "mu", mu)
        sub(ode, "mu2", mu if mu2 is None else mu2)
        if fdir1 is not None:
            sub(ode, "fdir1", fdir1)
    return element


def visual(link_element, name, geometry, xyzrpy=(0, 0, 0), color=WHITE, cast_shadows=True, **material_args):
    element = sub(link_element, "visual", name=f"{name}_visual")
    pose(element, xyzrpy)
    geometry(sub(element, "geometry"))
    if not cast_shadows:
        sub(element, "cast_shadows", False)
    material(element, color, **material_args)
    return element


def shape(link_element, name, geometry, xyzrpy=(0, 0, 0), color=WHITE, collide=True, mu=None, **material_args):
    """A visual and, unless collide is False, a matching collision."""
    if collide:
        collision(link_element, name, geometry, xyzrpy, mu=mu)
    return visual(link_element, name, geometry, xyzrpy, color, **material_args)


def joint(model, name, kind, parent, child, axis=(0, 0, 1), lower=None, upper=None, effort=None,
          velocity=None, damping=None, friction=None, stiffness=None, reference=None, xyzrpy=None):
    """A joint; xyzrpy is in the child frame. stiffness/reference make a spring."""
    element = sub(model, "joint", name=name, type=kind)
    if xyzrpy is not None:
        pose(element, xyzrpy)
    sub(element, "parent", parent)
    sub(element, "child", child)
    if kind in ("fixed", "ball"):
        return element
    ax = sub(element, "axis")
    sub(ax, "xyz", axis)
    if lower is not None or upper is not None:
        limit = sub(ax, "limit")
        sub(limit, "lower", -1e16 if lower is None else lower)
        sub(limit, "upper", 1e16 if upper is None else upper)
        if effort is not None:
            sub(limit, "effort", effort)
        if velocity is not None:
            sub(limit, "velocity", velocity)
    if any(v is not None for v in (damping, friction, stiffness)):
        dynamics = sub(ax, "dynamics")
        if damping is not None:
            sub(dynamics, "damping", damping)
        if friction is not None:
            sub(dynamics, "friction", friction)
        if stiffness is not None:
            sub(dynamics, "spring_stiffness", stiffness)
            sub(dynamics, "spring_reference", 0.0 if reference is None else reference)
    return element


def plugin(parent, filename, name, **params):
    element = sub(parent, "plugin", filename=filename, name=name)
    for key, value in params.items():
        for v in value if isinstance(value, list) else [value]:
            sub(element, key, v)
    return element


# --- Documents and model directories ---------------------------------------------

def model_root(name, static=False):
    """A new <sdf><model> tree; returns (root, model)."""
    root = ET.Element("sdf", version=SDF_VERSION)
    model = sub(root, "model", name=name)
    if static:
        sub(model, "static", True)
    return root, model


def document(root):
    ET.indent(root)
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"


def model_config(name, description, generator):
    """The model.config document of a generated model."""
    config = ET.Element("model")
    sub(config, "name", name)
    sub(config, "version", "1.0")
    sub(config, "sdf", "model.sdf", version=SDF_VERSION)
    sub(config, "description", f"{description} Generated by {generator}.")
    return document(config)


def write_model(models_dir, name, root, description, generator="sim/gen_worlds.py"):
    """Write models_dir/name/{model.sdf, model.config}; returns the directory."""
    directory = Path(models_dir) / name
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "model.sdf").write_text(document(root))
    (directory / "model.config").write_text(model_config(name, description, generator))
    return directory


def model_uri(name, *parts):
    return "/".join(("model:/", name) + parts)


def matrix_to_rpy(R):
    """Fixed-axis roll, pitch, yaw (SDF convention) of a rotation matrix."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2][0])))
    if abs(math.cos(pitch)) > 1e-9:
        return math.atan2(R[2][1], R[2][2]), pitch, math.atan2(R[1][0], R[0][0])
    return math.atan2(-R[1][2], R[1][1]), pitch, 0.0


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))
