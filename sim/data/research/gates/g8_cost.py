"""G8 (part 2): drivetrain cost, load_source contact vs joint, contact customisation on vs off (Delivery,
Dantzig, sensors stripped, CPU time per step by the method of spec 10.3)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gates_lib as G  # noqa: E402

world = G.REPO / "sim" / "worlds" / "urc_delivery.sdf"
env = G.env_for(plugin_dirs=[G.PROTO_BUILD])
cases = {"diffdrive": (G.world_variant(world, "g8_dd"), env)}
for source in ("contact", "joint"):
    for custom in (True, False):
        label = f"{source}_{'custom' if custom else 'nocustom'}"
        uri = G.proto_rover(f"g8_{label}", load_source=source, contact=custom)
        cases[label] = (G.world_variant(world, f"g8_{label}", rover_uri=uri), env)
runs = int(sys.argv[1]) if len(sys.argv) > 1 else 5
result = G.interleaved(cases, 20_000, runs)
base = result["diffdrive"]["cpu_ms_per_step"]
for label, r in result.items():
    r["vs_diffdrive"] = round(r["cpu_ms_per_step"] / base - 1, 3)
G.save("g8_cost", result)
