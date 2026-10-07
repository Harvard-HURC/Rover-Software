"""Build rover model variants from the snapshot of sim/gen_model.py (temp dir only)."""
import importlib.util, xml.etree.ElementTree as ET
from pathlib import Path
HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("gm", HERE / "gen_model.py")
gm = importlib.util.module_from_spec(spec); spec.loader.exec_module(gm)

def add_tire_compliance(model, k_lat, c_lat, k_rad=None, c_rad=None, hub_mass=0.1):
    """Insert a hub link between rocker and wheel: prismatic along the axle (tire lateral
    compliance) and optionally a radial (vertical, rocker frame z) prismatic."""
    links = {l.get("name"): l for l in model.findall("link")}
    for j in list(model.findall("joint")):
        n = j.get("name")
        if not (n.startswith("wheel_") and n.endswith("_joint")):
            continue
        w = n[len("wheel_"):-len("_joint")]
        wheel = links[f"wheel_{w}"]
        pose = wheel.find("pose").text
        parent = j.find("parent").text
        chain = [("lat", (0, 1, 0), k_lat, c_lat)]
        if k_rad:
            chain.append(("rad", (0, 0, 1), k_rad, c_rad))
        prev = parent
        for tag, axis, k, c in chain:
            hub = ET.Element("link", name=f"hub_{tag}_{w}")
            ET.SubElement(hub, "pose").text = pose
            inertial = ET.SubElement(hub, "inertial"); ET.SubElement(inertial, "mass").text = str(hub_mass)
            inertia = ET.SubElement(inertial, "inertia")
            for key in ("ixx", "iyy", "izz"):
                ET.SubElement(inertia, key).text = "1e-4"
            model.insert(list(model).index(wheel), hub)
            pj = ET.Element("joint", name=f"tire_{tag}_{w}", type="prismatic")
            ET.SubElement(pj, "parent").text = prev; ET.SubElement(pj, "child").text = f"hub_{tag}_{w}"
            ax = ET.SubElement(pj, "axis"); ET.SubElement(ax, "xyz").text = " ".join(map(str, axis))
            lim = ET.SubElement(ax, "limit"); ET.SubElement(lim, "lower").text = "-0.03"; ET.SubElement(lim, "upper").text = "0.03"
            dyn = ET.SubElement(ax, "dynamics"); ET.SubElement(dyn, "damping").text = str(c)
            ET.SubElement(dyn, "spring_reference").text = "0"; ET.SubElement(dyn, "spring_stiffness").text = str(k)
            model.insert(list(model).index(j), pj)
            prev = f"hub_{tag}_{w}"
        j.find("parent").text = prev


def build(name, params=None, drive="probe", plugin_opts=None, keep_diffdrive=True, effort=None,
          strip_sensors=True, tire=None):
    p = gm.Params(**(params or {}))
    root = ET.fromstring(gm.build_sdf(p).split("\n", 1)[1])
    model = root.find("model")
    if strip_sensors:
        for link in model.findall("link"):
            for s in link.findall("sensor"):
                if s.get("type") in ("rgbd_camera", "navsat"):
                    link.remove(s)
    for pl in model.findall("plugin"):
        if pl.get("name") == "gz::sim::systems::DiffDrive" and not keep_diffdrive:
            model.remove(pl)
    if effort is not None:
        for j in model.findall("joint"):
            if j.get("name").startswith("wheel_"):
                j.find("axis/limit/effort").text = str(effort)
    if tire:
        add_tire_compliance(model, **tire)
    opts = dict(drive=drive); opts.update(plugin_opts or {})
    pl = ET.SubElement(model, "plugin", filename="RoverDrive", name="rover_sim::RoverDrive")
    for k, v in opts.items():
        if k == "zones":
            for z in v:
                ET.SubElement(pl, "zone").text = " ".join(str(x) for x in z)
        else:
            ET.SubElement(pl, k).text = str(v).lower() if isinstance(v, bool) else str(v)
    d = HERE / "variants" / name / "rover"
    d.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    (d / "model.sdf").write_text('<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode"))
    (d / "model.config").write_text('<?xml version="1.0"?><model><name>rover</name><version>0.1</version>'
                                    '<sdf version="1.11">model.sdf</sdf></model>')
    return str(d.parent)
