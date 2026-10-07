"""G7: Delivery and Astrobiology at 2049^2 vs today's 1025^2: CPU time per step (driving, sensors
stripped, RTF 0). Generation time and invariants are checked separately. Usage: python g7.py <runs>"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_lib as G  # noqa: E402

runs = int(sys.argv[1]) if len(sys.argv) > 1 else 5
tree = G.SCRATCH / "trees" / "g7_2049" / "sim"
result = {}
for world in ("urc_delivery", "urc_astrobiology"):
    cases = {f"{world}/1025": (G.world_variant(G.REPO / "sim" / "worlds" / f"{world}.sdf", f"g7_{world}_1025", rtf=0),
                               G.env_for()),
             f"{world}/2049": (G.world_variant(tree / "worlds" / f"{world}.sdf", f"g7_{world}_2049", rtf=0),
                               G.env_for(models_dir=tree / "models"))}
    r = G.interleaved(cases, 20_000, runs)
    r[f"{world}/2049"]["rtf_loss"] = round(r[f"{world}/2049"]["cpu_ms_per_step"] / r[f"{world}/1025"]["cpu_ms_per_step"] - 1, 3)
    result.update(r)
result["generation_s"] = json.loads((tree.parent / "generation.json").read_text())["seconds"]
G.save("g7_cost", result)
