"""G5: drivetrain prototype + PGS per world, CPU time per step (method of spec 10.3), against the
prototype with Dantzig and today's DiffDrive rover with Dantzig. Sensors stripped, RTF 0.
Usage: python g5.py <load_source> <runs> [world ...]"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_lib as G  # noqa: E402

source = sys.argv[1]
runs = int(sys.argv[2])
WORLDS = ["proving_ground", "urc_delivery", "urc_astrobiology", "urc_autonomy", "urc_equipment_servicing",
          "rover_test", "flat"]
worlds = sys.argv[3:] or WORLDS
env = G.env_for(plugin_dirs=[G.PROTO_BUILD])
proto = G.proto_rover(f"g5_{source}", load_source=source)
flat = G.VARIANTS / "g5_flat_src.sdf"
flat.write_text(G.S.world_sdf())
result = {}
for world in worlds:
    src = flat if world == "flat" else G.REPO / "sim" / "worlds" / f"{world}.sdf"
    cases = {f"{world}/diffdrive_dantzig": (G.world_variant(src, f"g5_{world}_dd", rtf=0), env),
             f"{world}/proto_dantzig": (G.world_variant(src, f"g5_{world}_pd", rover_uri=proto, rtf=0), env),
             f"{world}/proto_pgs": (G.world_variant(src, f"g5_{world}_pp", rover_uri=proto, solver="pgs", rtf=0), env)}
    r = G.interleaved(cases, 20_000, runs)
    base = r[f"{world}/diffdrive_dantzig"]["cpu_ms_per_step"]
    for label, v in r.items():
        v["vs_diffdrive_dantzig"] = round(v["cpu_ms_per_step"] / base - 1, 3)
        v["passes_1_1x"] = v["real_time_factor_cpu"] >= 1.1
    result.update(r)
    G.save(f"g5_{source}{os.environ.get('G5_TAG', '')}", result)
