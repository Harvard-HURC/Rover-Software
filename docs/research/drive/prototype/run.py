"""Launch harness runs in subprocesses."""
import json, subprocess, sys, os
from pathlib import Path
HERE = Path(__file__).resolve().parent
PY = "/Users/alarion239/Desktop/Rover/.pixi/envs/default/bin/python"

def flat_world(name="flat", mu=None, extra="", spawn=(0, 0, 0.02, 0), physics_extra=""):
    surf = ""
    if mu is not None:
        surf = f"<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>"
    sdf = f"""<?xml version="1.0"?>
<sdf version="1.11">
  <world name="test">
    <physics name="1ms" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>0</real_time_factor>{physics_extra}</physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <model name="ground"><static>true</static><link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>{surf}</collision>
    </link></model>
    {extra}
    <include><uri>model://rover</uri><name>rover</name><pose>{spawn[0]} {spawn[1]} {spawn[2]} 0 0 {spawn[3]}</pose></include>
  </world>
</sdf>"""
    p = HERE / "worlds" / f"{name}.sdf"
    p.write_text(sdf)
    return str(p)

def run(name, world, schedule, seconds, mode="diffdrive", models=(), motor=None, plugins=(), quiet=True):
    cfg = dict(world=world, schedule=schedule, seconds=seconds, mode=mode, models=list(models),
               plugins=list(plugins), motor=motor or {}, out=str(HERE / "logs" / f"{name}.npz"))
    cp = HERE / "logs" / f"{name}.json"
    cp.write_text(json.dumps(cfg))
    r = subprocess.run([PY, str(HERE / "harness.py"), str(cp)], capture_output=True, text=True)
    if r.returncode != 0 or "saved" not in r.stdout:
        print(r.stdout[-2000:], r.stderr[-4000:])
        raise RuntimeError(name)
    errs = [l for l in r.stderr.splitlines() if "[Err]" in l or "rror" in l]
    if not quiet:
        print("\n".join(l for l in r.stdout.splitlines()+r.stderr.splitlines() if "DUMP" in l))
    if errs and not quiet:
        print("\n".join(errs[:10]))
    return cfg["out"]
