"""G8 (part 3): contact vs joint load source, customisation on, more interleaved pairs (Delivery,
Dantzig, sensors stripped, RTF 0). Usage: python g8_ab.py <runs>"""
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_lib as G  # noqa: E402

runs = int(sys.argv[1]) if len(sys.argv) > 1 else 10
world = G.REPO / "sim" / "worlds" / "urc_delivery.sdf"
env = G.env_for(plugin_dirs=[G.PROTO_BUILD])
cases = {}
for source in ("contact", "joint"):
    uri = G.proto_rover(f"g8ab_{source}", load_source=source)
    cases[source] = (G.world_variant(world, f"g8ab_{source}", rover_uri=uri, rtf=0), env)
result = G.interleaved(cases, 20_000, runs)
pairs = [j / c for c, j in zip(result["contact"]["cpu_ms_per_step_runs"], result["joint"]["cpu_ms_per_step_runs"])]
result["joint_over_contact"] = dict(median=round(statistics.median(pairs), 3), pairs=[round(p, 3) for p in pairs],
                                    joint_cheaper_in=sum(p < 1 for p in pairs), of=len(pairs))
G.save("g8_ab", result)
