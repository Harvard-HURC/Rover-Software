"""Shared helpers of the WS-0 gate runs (research code, not part of the simulation; design spec 10.4,
results in ../gates.json).

Everything a run writes (model variants, world copies, raw results, the prototype builds of build.sh)
goes to GATES_SCRATCH (default: <tmp>/rover_gates), never into the repo. The repo is the one these
scripts lie in, with its generated worlds (python sim/gen_model.py; python sim/gen_worlds.py) and its
plugins built in sim/build.
"""
import json
import os
import re
import statistics
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
SCRATCH = Path(os.environ.get("GATES_SCRATCH", Path(tempfile.gettempdir()) / "rover_gates"))
PROTO_BUILD = SCRATCH / "proto" / "build"  # the drivetrain prototype with load_source (build.sh)
FLY_BUILD = SCRATCH / "flycam" / "build"  # the fly camera prototype (build.sh)
sys.path.insert(0, str(REPO / "sim" / "tests"))
os.environ["PATH"] = str(Path(sys.prefix) / "bin") + os.pathsep + os.environ["PATH"]

import simulate as S  # noqa: E402  (the gzenv environment and a private partition)
import gzenv  # noqa: E402
import gen_model  # noqa: E402
from worldfiles import SENSORS  # noqa: E402

RESULTS = SCRATCH / "results"
RESULTS.mkdir(parents=True, exist_ok=True)
VARIANTS = SCRATCH / "variants"
VARIANTS.mkdir(exist_ok=True)

# The prototype's "REAL" contact settings (exp_rtf.py), plus rolling resistance as a hub force and slip.
PROTO_OPTS = dict(drive="motor", contact=True, friction_circle=True, perp_ratio=0.0, mu_s=1.0, mu_k=0.8,
                  v_stribeck=0.03, mu_noise=0.2, mu_noise_scale=0.3, rolling_resistance=0.1, rr_mode="force",
                  slip_compliance=0.3, load_source="contact")


def proto_rover(name, **opts):
    """A rover model (today's gen_model geometry) driven by the gate copy of the prototype plugin instead
    of DiffDrive, as the design's physical variant (D22: never both): wheel effort 1000, velocity limit
    16, isotropic tyre mu 1. Returns its file URI."""
    root = ET.fromstring(gen_model.build_sdf(gen_model.Params()).split("\n", 1)[1])
    model = root.find("model")
    for plugin in model.findall("plugin"):
        if plugin.get("name") == "gz::sim::systems::DiffDrive":
            model.remove(plugin)
    for joint in model.findall("joint"):
        if joint.get("name").startswith("wheel_"):
            joint.find("axis/limit/effort").text = "1000"
            joint.find("axis/limit/velocity").text = "16"
    for collision in model.iter("collision"):
        if collision.get("name") == "tire_collision":
            ode = collision.find("surface/friction/ode")
            ode.find("mu").text = "1"
            ode.find("mu2").text = "1"
            ode.remove(ode.find("fdir1"))
    settings = dict(PROTO_OPTS, **opts)
    plugin = ET.SubElement(model, "plugin", filename="RoverDrive", name="rover_sim::RoverDrive")
    for key, value in settings.items():
        ET.SubElement(plugin, key).text = str(value).lower() if isinstance(value, bool) else str(value)
    directory = VARIANTS / name / "rover"
    directory.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    (directory / "model.sdf").write_text('<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode"))
    (directory / "model.config").write_text('<?xml version="1.0"?><model><name>rover</name><version>1</version>'
                                            '<sdf version="1.11">model.sdf</sdf></model>')
    return directory.as_uri()


def world_variant(src, name, rover_uri=None, solver=None, strip_sensors=True, rtf=None, extra=""):
    """A copy of world file `src` in VARIANTS (sensors stripped by default)."""
    text = Path(src).read_text()
    if strip_sensors:
        text = SENSORS.sub("", text)
    text = S.variant_sdf(text, rover_uri, solver)
    if rtf is not None:
        text = re.sub(r"<real_time_factor>[^<]*</real_time_factor>", f"<real_time_factor>{rtf}</real_time_factor>",
                      text, count=1)
    if extra:
        text = text.replace("</world>", extra + "\n  </world>", 1)
    path = VARIANTS / f"{name}.sdf"
    path.write_text(text)
    return path


def env_for(models_dir=None, plugin_dirs=()):
    """The gzenv environment, with another models directory first and more plugin directories last."""
    env = gzenv.environment()
    if models_dir is not None:
        env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join([str(models_dir), env["GZ_SIM_RESOURCE_PATH"]])
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join([env["GZ_SIM_SYSTEM_PLUGIN_PATH"], *map(str, plugin_dirs)])
    return env


def interleaved(cases, iterations, runs, schedule=S.DRIVE_SCHEDULE):
    """cases {label: (world path, env)}: simulate.cpu_time_per_step's method (spec 10.3), but each case
    with its own environment (a scratch tree's models, the prototype plugins). Returns {label: dict}."""
    names = {S.world_name(path) for path, _ in cases.values()}
    rows = {label: [] for label in cases}
    with S.twist_publisher(names, schedule):
        for k in range(runs):
            for label, (path, env) in cases.items():
                load = os.getloadavg()[0]
                startup = S.gz_run(path, 1, env)
                full = S.gz_run(path, iterations, env)
                rows[label].append((load, startup, full))
                print(f"run {k} {label}: {(full[0] - startup[0]) / (iterations - 1) * 1e3:.4f} ms/step "
                      f"(startup {startup[1]:.1f} s wall, rss {full[2] / 2**20:.0f} MB, load {load:.1f})", flush=True)
    out = {}
    for label, r in rows.items():
        steps = [(full[0] - startup[0]) / (iterations - 1) for _, startup, full in r]
        med = statistics.median(steps)
        out[label] = dict(cpu_ms_per_step=round(med * 1e3, 4), cpu_ms_per_step_runs=[round(s * 1e3, 4) for s in steps],
                          real_time_factor_cpu=round(0.001 / med, 3),
                          startup_wall_s=round(statistics.median(x[1][1] for x in r), 2),
                          startup_cpu_s=round(statistics.median(x[1][0] for x in r), 2),
                          full_wall_s=round(statistics.median(x[2][1] for x in r), 2),
                          peak_rss_mb=round(max(x[2][2] for x in r) / 2**20, 1),
                          load_avg_1min=[round(x[0], 2) for x in r])
    return out


def save(name, data):
    path = RESULTS / f"{name}.json"
    path.write_text(json.dumps(data, indent=1) + "\n")
    print(f"saved {path}")
