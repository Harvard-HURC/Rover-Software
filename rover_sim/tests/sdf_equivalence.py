"""Compare two SDF models number by number (test_equivalence.Equivalence: the generated rover against the frozen one
gen_model.py wrote, tests/fixtures/rover_model).

differences(reference, generated) lists every difference: an element or attribute that one has and the other lacks,
a number that differs beyond the tolerance (a pose by its position and its rotation matrix), different text, a
different order of links, joints or plugins. Children with the same tag pair by name where they all have one (not
plugins: those pair in order), otherwise in order; children with different tags may come in any order, as SDF reads
them. An element that one side lacks counts as SDF's default: DEFAULTS lists the values that the converter writes
where gen_model.py wrote nothing, which sdformat assumes when the element is missing."""
import math
import xml.etree.ElementTree as ET

import numpy as np

REL_TOL = 1e-9  # 9 significant digits, as gen_model.py and `gz sdf -p --precision 9` write numbers
# Numbers this close are the same: a position 1e-12 m apart, or an SDF float (particle_scatter_ratio) that
# the converter writes as its float32 value.
ABS_TOL = 1e-12
ROTATION_TOL = 1e-8  # entries of two rotation matrices: an angle to pi written with 9 significant digits
# The tail of an element's tag path -> SDF 1.11's default for it.
DEFAULTS = {
    ("axis", "dynamics", "damping"): "0",
    ("axis", "dynamics", "friction"): "0",
    ("axis", "dynamics", "spring_reference"): "0",
    ("axis", "dynamics", "spring_stiffness"): "0",
    ("axis", "limit", "effort"): "-1",
    ("axis", "limit", "velocity"): "-1",
    ("inertial", "pose"): "0 0 0 0 0 0",
    ("collision", "pose"): "0 0 0 0 0 0",
    ("visual", "pose"): "0 0 0 0 0 0",
    ("sensor", "pose"): "0 0 0 0 0 0",
    ("depth_camera", "output"): "depths",
}
ORDERED = ("link", "joint", "plugin")  # model children whose order counts (entity creation, the systems' order)


def differences(reference, generated):
    """Every difference between two SDF documents' <model>s, as lines of text (none: the same model)."""
    a, b = ET.fromstring(reference).find("model"), ET.fromstring(generated).find("model")
    out = []
    for tag in ORDERED:
        names_a = [e.get("name") for e in a.findall(tag)]
        names_b = [e.get("name") for e in b.findall(tag)]
        if names_a != names_b:
            out.append(f"model/{tag} order: {names_a} != {names_b}")
    _compare(a, b, ("model",), out)
    return out


def _numbers(text):
    try:
        return [float(v) for v in text.split()]
    except ValueError:
        return None


def _rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _close(x, y):
    return math.isclose(x, y, rel_tol=REL_TOL, abs_tol=ABS_TOL)


def _text_differs(path, ta, tb):
    ta, tb = (ta or "").strip(), (tb or "").strip()
    if ta == tb:
        return False
    na, nb = _numbers(ta), _numbers(tb)
    if na is None or nb is None or len(na) != len(nb):
        return True
    if path[-1] == "pose" and len(na) == 6:
        if not all(_close(x, y) for x, y in zip(na[:3], nb[:3])):
            return True
        return not np.allclose(_rotation(na[3:]), _rotation(nb[3:]), rtol=0.0, atol=ROTATION_TOL)
    return not all(x == y or _close(x, y) for x, y in zip(na, nb))


def _is_default(path, element):
    """Whether `element`, which one side lacks, holds only SDF defaults."""
    children = list(element)
    if children:
        return all(_is_default(path + (c.tag,), c) for c in children)
    default = next((v for k, v in DEFAULTS.items() if path[-len(k):] == k), None)
    return default is not None and not _text_differs(path, element.text, default)


def _compare(a, b, path, out):
    where = "/".join(path)
    if dict(a.attrib) != dict(b.attrib):
        out.append(f"{where}: attributes {dict(a.attrib)} != {dict(b.attrib)}")
    if _text_differs(path, a.text, b.text):
        out.append(f"{where}: {(a.text or '').strip()!r} != {(b.text or '').strip()!r}")
    for tag in dict.fromkeys([c.tag for c in a] + [c.tag for c in b]):
        ca, cb = a.findall(tag), b.findall(tag)
        if tag != "plugin" and all(c.get("name") is not None for c in ca + cb):
            pairs = {c.get("name"): [c, None] for c in ca}
            for c in cb:
                pairs.setdefault(c.get("name"), [None, None])[1] = c
            items = [(f"{tag}[{name}]", x, y) for name, (x, y) in pairs.items()]
        else:
            ca, cb = ca + [None] * (len(cb) - len(ca)), cb + [None] * (len(ca) - len(cb))
            items = [(f"{tag}[{i}]", x, y) for i, (x, y) in enumerate(zip(ca, cb))]
        for label, x, y in items:
            if x is None or y is None:
                present = x if y is None else y
                if not _is_default(path + (tag,), present):
                    side = "reference" if y is None else "generated"
                    text = ET.tostring(present, encoding="unicode").strip()[:120]
                    out.append(f"{where}/{label}: only in the {side} model ({text})")
                continue
            _compare(x, y, path + (tag,), out)
