"""G1: Autonomy on the 0.5 m lidar at 4097^2 vs its 2049^2 twin (same clutter), 60 s driving runs,
CPU time per step, start-up wall time and peak memory; today's 3DEP world for reference.
Sensors stripped, RTF 0. Usage: python g1.py <runs>"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_lib as G  # noqa: E402

runs = int(sys.argv[1]) if len(sys.argv) > 1 else 5
schedule = tuple((t + 24.0 * k, vx, wz) for k in range(3) for t, vx, wz in G.S.DRIVE_SCHEDULE)
cases = {}
for label, tree in (("lidar_4097", G.SCRATCH / "trees" / "g1_lidar4097" / "sim"),
                    ("lidar_2049", G.SCRATCH / "trees" / "g1_lidar2049" / "sim"),
                    ("today_3dep_2049", G.REPO / "sim")):
    world = G.world_variant(tree / "worlds" / "urc_autonomy.sdf", f"g1_{label}", rtf=0)
    cases[label] = (world, G.env_for(models_dir=tree / "models"))
r = G.interleaved(cases, 60_000, runs, schedule)
base = r["lidar_2049"]["cpu_ms_per_step"]
for label, v in r.items():
    v["vs_lidar_2049"] = round(v["cpu_ms_per_step"] / base - 1, 3)
r["gate"] = dict(rtf_loss=r["lidar_4097"]["vs_lidar_2049"], startup_wall_s=r["lidar_4097"]["startup_wall_s"],
                 peak_rss_mb=r["lidar_4097"]["peak_rss_mb"])
r["gate"]["passes"] = (r["gate"]["rtf_loss"] <= 0.15 and r["gate"]["startup_wall_s"] <= 30
                       and r["gate"]["peak_rss_mb"] <= 3.5 * 1024)
G.save("g1", r)
