#!/usr/bin/env python3
"""Generate the rover's Gazebo model, models/rover/model.sdf, from its description: urdf/rover_sim.urdf.xacro
(rover_description's rover.urdf.xacro and Gazebo's parts) expanded by xacro, converted to SDF by `gz sdf -p` and
post-processed (postprocess), so that the converter's quirks leave no trace; with it the drivetrain's ground surfaces
(config/surfaces.yaml, from urc/terrains.py), the dust sprite, a copy of the tread tyre's mesh and the driver
station's cameras (viewers.py). `pixi run sim-model` runs it.

build_sdf(variant) builds a variant of the rover (rover_config.Variant: some of rover.yaml's or sim.yaml's numbers
changed) without writing anything; the tests drive such variants. tests/test_equivalence.py checks that the default
rover and the variants the tests use are the frozen models gen_model.py wrote (tests/fixtures/rover_model).
"""
import dataclasses
import functools
import os
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
import rover_config  # noqa: E402
from rover_config import DUST_SPRITE, SIM, WHEEL_MESH, Variant  # noqa: E402
from urc import sdf  # noqa: E402  (the SDF writer every generated model shares)

XACRO = SIM_DIR / "urdf" / "rover_sim.urdf.xacro"
SURFACES_YAML = SIM_DIR / "config" / "surfaces.yaml"
MODELS_DIR = SIM_DIR / "models"
MODEL_DIR = MODELS_DIR / "rover"
DESCRIPTION_MESH = rover_config.DESCRIPTION_DIR / WHEEL_MESH  # the tread tyre (rover_description/meshes)
CONVERTED_MESHES = "model://rover_description/meshes/"  # how the converter writes the description's package:// URIs
LUMP = "fixed_joint_lump__"  # sdformat's marker in the names of shapes it moved across fixed joints
# What the converter writes into every joint a <gazebo> block names: SDF's defaults for ODE's joint limits.
JOINT_PHYSICS_DEFAULTS = {"ode/limit/cfm": "0", "ode/limit/erp": "0.2"}
SURFACES_HEADER = ("# The drivetrain's ground surfaces: every urc/terrains.py type's traction under its dig-in preset\n"
                   "# (terrains.DIG) and its dust factor, keyed by type. Written by tools/gen_rover.py\n"
                   "# (pixi run sim-model) from the catalogue; do not edit.\n")


def surface_rows():
    """Every catalogue ground type's traction (terrains.TYPES, design spec 5.6, under the dig-in preset terrains.DIG,
    as worlds write it) and dust factor: the drivetrain's <surface> rows, keyed by type, for config/surfaces.yaml.
    The model takes the rows of sim.yaml's default and object surfaces; a key the catalogue lacks has none, and the
    drivetrain reports it when it starts and grips there as plain Coulomb mu 1."""
    from urc import terrains  # here: the catalogue imports the texture generators
    return {key: {**dataclasses.asdict(terrains.traction(kind)), "dust": kind.appearance.dust}
            for key, kind in terrains.TYPES.items()}


def surfaces_text():
    """config/surfaces.yaml as surface_rows() makes it."""
    return SURFACES_HEADER + yaml.safe_dump(surface_rows(), sort_keys=False)


def build_urdf(variant=Variant()):
    """The variant's URDF: rover_sim.urdf.xacro expanded with its numbers."""
    import xacro  # here: only building a model needs it
    with tempfile.TemporaryDirectory(prefix="gen_rover_") as tmp:
        rover_yaml, sim_yaml = Path(tmp) / "rover.yaml", Path(tmp) / "sim.yaml"
        rover_yaml.write_text(yaml.safe_dump(variant.rover.plain(), sort_keys=False))
        sim_yaml.write_text(yaml.safe_dump(variant.sim.plain(), sort_keys=False))
        doc = xacro.process_file(str(XACRO), mappings={
            "description_dir": str(rover_config.DESCRIPTION_DIR), "rover_yaml": str(rover_yaml),
            "sim_yaml": str(sim_yaml), "surfaces_yaml": str(SURFACES_YAML)})
    return doc.toprettyxml(indent="  ")


@functools.lru_cache(maxsize=64)
def build_sdf(variant=Variant()):
    """The variant's model as an SDF 1.11 document: build_urdf, `gz sdf -p`, postprocess."""
    mode = variant.sim.drivetrain.mode
    if mode not in ("physical", "diffdrive"):
        raise ValueError(f"sim.yaml drivetrain mode {mode!r}: 'physical' or 'diffdrive'")
    urdf = build_urdf(variant)
    with tempfile.TemporaryDirectory(prefix="gen_rover_") as tmp:
        path = Path(tmp) / "rover.urdf"
        path.write_text(urdf)
        result = subprocess.run(["gz", "sdf", "-p", "--precision", "9", str(path)], capture_output=True, text=True)
    if result.returncode != 0 or "<sdf" not in result.stdout:
        raise RuntimeError(f"gz sdf -p failed:\n{result.stdout}\n{result.stderr}")
    return postprocess(result.stdout, urdf, variant.sim.continuous_stops)


# --- The converter's quirks ----------------------------------------------------------------------------------------

def postprocess(text, urdf, stops):
    """The SDF that `gz sdf -p` writes for the rover's URDF, written as the model gen_model.py wrote (the
    equivalence test compares them number by number). sdformat's URDF conversion (parser_urdf.cc):
    - merges every link that hangs from a fixed joint (the sensor and LED frames) into its parent and adds a <frame>
      for it and for its joint: the frames are dropped (they are the description's, for ROS; the sensors and the
      LED's visual already sit in their link with their poses);
    - names a shape "<link>_[fixed_joint_lump__]<name>_<collision|visual>[_<n>]": back to "<name>_<...>";
    - writes a URDF colour rgba as diffuse and ambient rgb / 0.8, clipped at 1: both back to the URDF's rgba;
    - writes each link's pose relative to its joint and each joint's relative to its parent link: links get their
      model-frame poses and joints none (a URDF joint's frame is its child link's, SDF's default);
    - orders links and joints by walking the tree, children by joint name: back to the URDF's order;
    - leaves a continuous joint's stops infinite: +-stops (sim.yaml continuous_stops), as before;
    - passes plugin parameters through as xacro wrote them (Python's str): numbers with 9 significant digits, as
      --precision 9 writes SDF's own and gen_model.py wrote them;
    - adds a <physics> with ODE's default limits (JOINT_PHYSICS_DEFAULTS) to every joint a <gazebo> block names (the
      tyre model's springs): dropped; anything else there is an error;
    - leaves empty <surface> scaffolding in collisions without friction: dropped;
    - turns the description's package:// mesh URIs into model://rover_description/...: model://rover/..., where
      main() copies the meshes."""
    root = ET.fromstring(text)
    model = root.find("model")
    urdf_root = ET.fromstring(urdf)
    for frame in model.findall("frame"):
        model.remove(frame)
    colors = _urdf_colors(urdf_root)
    for link in model.findall("link"):
        name = link.get("name")
        for shape in [*link.findall("collision"), *link.findall("visual")]:
            match = re.fullmatch(rf"{re.escape(name)}_(?:{LUMP})?(.+_(?:collision|visual))(?:_\d+)?", shape.get("name"))
            if match is None:
                raise ValueError(f"unexpected shape name {shape.get('name')!r} in link {name!r}")
            shape.set("name", match.group(1))
        for visual in link.findall("visual"):
            material = visual.find("material")
            if material is not None:
                rgba = colors[name, visual.get("name")]
                for tag in ("ambient", "diffuse"):
                    material.find(tag).text = sdf.fmt(rgba)
        for uri in link.iter("uri"):
            if uri.text.startswith(CONVERTED_MESHES):
                uri.text = sdf.model_uri(MODEL_DIR.name, "meshes", uri.text[len(CONVERTED_MESHES):])
        for collision in link.findall("collision"):
            for surface in collision.findall("surface"):
                _drop_empty(collision, surface)
    _resolve_poses(model)
    for joint in model.findall("joint"):
        for physics in joint.findall("physics"):
            found = {"/".join(_path(physics, e)): (e.text or "").strip() for e in physics.iter() if not len(e)}
            if found != JOINT_PHYSICS_DEFAULTS:
                raise ValueError(f"joint {joint.get('name')!r}: unexpected <physics> {found}")
            joint.remove(physics)
        limit = joint.find("axis/limit")
        if limit is not None and limit.findtext("lower") == "-inf" and limit.findtext("upper") == "inf":
            limit.find("lower").text, limit.find("upper").text = sdf.fmt(-stops), sdf.fmt(stops)
    for plugin in model.findall("plugin"):
        for element in plugin.iter():
            element.text = _number_text(element.text)
    order = [e.get("name") for e in urdf_root if e.tag in ("link", "joint")]
    links = sorted(model.findall("link"), key=lambda e: order.index(e.get("name")))
    joints = sorted(model.findall("joint"), key=lambda e: order.index(e.get("name")))
    rest = [e for e in model if e.tag not in ("link", "joint")]
    for element in list(model):
        model.remove(element)
    model.extend(links + joints + rest)
    root.attrib = {"version": sdf.SDF_VERSION}
    return sdf.document(root)


def _path(top, element):
    """The tags from below `top` down to `element`."""
    parents = {child: parent for parent in top.iter() for child in parent}
    tags = []
    while element is not top:
        tags.append(element.tag)
        element = parents[element]
    return reversed(tags)


def _drop_empty(parent, element):
    """Remove `element` and its descendants that hold no text and no attributes (the converter's scaffolding)."""
    for child in list(element):
        _drop_empty(element, child)
    if not len(element) and not element.attrib and not (element.text or "").strip():
        parent.remove(element)


def _number_text(text):
    """A parameter's text with every number in 9 significant digits (sdf.fmt); other text unchanged."""
    if text is None or not text.strip():
        return text
    try:
        values = [float(v) for v in text.split()]
    except ValueError:
        return text
    return sdf.fmt(tuple(values) if len(values) > 1 else values[0])


def _urdf_colors(urdf_root):
    """(SDF link, SDF visual name) -> the URDF colour, for every URDF visual with a material: a link merged across
    fixed joints counts as the link it merged into, and a visual's SDF name is its URDF name (or its link's) +
    "_visual"."""
    named = {m.get("name"): m.find("color").get("rgba") for m in urdf_root.findall("material")
             if m.find("color") is not None}
    parent = {j.find("child").get("link"): (j.find("parent").get("link"), j.get("type"))
              for j in urdf_root.findall("joint")}

    def owner(link):
        while link in parent and parent[link][1] == "fixed":
            link = parent[link][0]
        return link

    colors = {}
    for link in urdf_root.findall("link"):
        for visual in link.findall("visual"):
            material = visual.find("material")
            if material is None:
                continue
            color = material.find("color")
            rgba = color.get("rgba") if color is not None else named[material.get("name")]
            key = (owner(link.get("name")), (visual.get("name") or link.get("name")) + "_visual")
            colors[key] = tuple(float(v) for v in rgba.split())
    return colors


def _pose_matrix(text):
    x, y, z, roll, pitch, yaw = (float(v) for v in (text or "0 0 0 0 0 0").split())
    m = np.eye(4)
    m[:3, :3] = sdf.rpy_to_matrix(roll, pitch, yaw)
    m[:3, 3] = (x, y, z)
    return m


def _resolve_poses(model):
    """Every link's pose in the model frame, from the converter's chain (a link relative to its joint, a joint
    relative to its parent link); the joints' poses dropped."""
    joints = {j.get("name"): j for j in model.findall("joint")}
    links = {link.get("name"): link for link in model.findall("link")}
    frames = {}

    def frame(name):  # a link's or joint's model-frame transform
        if name not in frames:
            element = links[name] if name in links else joints[name]
            pose = element.find("pose")
            if pose is None:
                frames[name] = np.eye(4)
            else:
                base = pose.get("relative_to")
                frames[name] = (frame(base) if base and base != "__model__" else np.eye(4)) @ _pose_matrix(pose.text)
        return frames[name]

    for name, link in links.items():
        m = frame(name)
        pose = link.find("pose")
        if pose is None:
            continue
        roll, pitch, yaw = sdf.matrix_to_rpy(m[:3, :3])
        pose.attrib.pop("relative_to", None)
        pose.text = sdf.fmt(tuple(float(v) + 0.0 for v in (*m[:3, 3], roll, pitch, yaw)))
    for joint in joints.values():
        pose = joint.find("pose")
        if pose is not None:
            joint.remove(pose)


# --- The files of the rover model ----------------------------------------------------------------------------------

def write_dust_sprite(path, alpha=SIM.dust.alpha):
    """The rover's dust sprite (DUST_SPRITE): textures.dust_puff in the catalogue's dust colour at sim.yaml's dust
    alpha. main() writes it whether or not dust is on, so the emitters find it when it is, and so do
    tests/test_render.py's."""
    from urc import terrains, textures  # here: the catalogue imports the texture generators
    textures.dust_puff(path, terrains.DUST_RGB, alpha)


def write_if_changed(path, data):
    """Write `data` (text or bytes) to `path` unless it holds those bytes already, through a hidden file and a rename:
    main() reads config/surfaces.yaml right after writing it, and two runs at once (pixi run test beside pixi run
    drive, both run sim-model) must never read a half-written file."""
    data = data.encode() if isinstance(data, str) else data
    if path.is_file() and path.read_bytes() == data:
        return
    partial = path.with_name(f".{path.name}.{os.getpid()}")
    partial.write_bytes(data)
    partial.replace(path)


def main():
    write_if_changed(SURFACES_YAML, surfaces_text())
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    out = MODEL_DIR / "model.sdf"
    write_if_changed(out, build_sdf(Variant()))
    sprite = MODEL_DIR / DUST_SPRITE
    sprite.parent.mkdir(parents=True, exist_ok=True)
    write_dust_sprite(sprite)
    wheel = MODEL_DIR / WHEEL_MESH
    wheel.parent.mkdir(parents=True, exist_ok=True)
    write_if_changed(wheel, DESCRIPTION_MESH.read_bytes())
    print(f"wrote {SURFACES_YAML.relative_to(SIM_DIR)}, {out.relative_to(SIM_DIR)}, "
          f"{sprite.relative_to(MODEL_DIR)} and {wheel.relative_to(MODEL_DIR)}")
    import viewers  # here: the station's cameras are the simulation's own models
    viewers.write_all(MODELS_DIR)


if __name__ == "__main__":
    main()
