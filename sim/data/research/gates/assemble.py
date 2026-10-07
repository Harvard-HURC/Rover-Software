"""Collect the gate results (GATES_SCRATCH/results/*.json) into sim/data/research/gates.json: each gate's
question, pass rule and fallback (design spec 10.4), what was measured, the verdict and the decision.
The verdicts are judged here from the numbers; the decisions and notes are written in DECISIONS after
reading them. Usage: python assemble.py"""
import json
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gates_lib as G  # noqa: E402

R = G.RESULTS
OUT = Path(__file__).resolve().parents[1] / "gates.json"


def load(name):
    path = R / f"{name}.json"
    return json.loads(path.read_text()) if path.exists() else None


def cost(entry, *keys):
    """The CPU-time fields of an interleaved() entry worth keeping."""
    keys = keys or ("cpu_ms_per_step", "real_time_factor_cpu", "cpu_ms_per_step_runs", "startup_wall_s", "peak_rss_mb")
    out = {k: entry[k] for k in keys if k in entry}
    out["load_avg_1min_median"] = round(statistics.median(entry["load_avg_1min"]), 2)
    return out


SPEC = {  # design spec 10.4, verbatim in substance
    "G1": dict(question="Autonomy at 4097^2 (collision + visual) on the 0.5 m lidar, with a 60 s driving run",
               rule="RTF loss <= 15 % vs 2049^2, start-up <= 30 s, memory <= 3.5 GB",
               fallback="2049^2 for both (1 m); real 0.5 m relief lost below 1 m"),
    "G2": dict(question="PGS on Equipment Servicing, Delivery and the proving ground",
               rule="Lander key-press test passes; toolbox + wrench stay put 60 s",
               fallback="Dantzig in that world (spec 6.6)"),
    "G3": dict(question="GLB meshes as collisions in DART; GLB vs OBJ visual memory",
               rule="Loads, contacts correct", fallback="OBJ for collisions, GLB for visuals"),
    "G4": dict(question="Alpha-tested textures from SDF (shrub cards, feathered decal edges)",
               rule="Visible cut-out / feathered edge", fallback="Opaque low-poly shrubs; no detail decals"),
    "G5": dict(question="Drivetrain prototype + PGS per world, method of spec 10.3",
               rule=">= 1.1x real time", fallback="Dantzig in that world; feeds Q2"),
    "G6": dict(question="Station frame rate: full-config render assets in Delivery and Equipment Servicing; eye and "
                        "fly watched, RGB-D subscribed, RTF 1",
               rule=">= 15 fps; LensFlare logs 'Render pass added' and disconnects",
               fallback="Lower shrub/pebble budgets; fly camera 960 x 540; lens flare off"),
    "G7": dict(question="Delivery and Astrobiology at 2049^2 (4x today's samples)",
               rule="Generation <= 2 min each; RTF loss <= 10 %; mission grades and easy-route invariants unchanged",
               fallback="1025^2 + relief transfer at 1 m"),
    "G8": dict(question="Drivetrain cost: load_source contact vs joint, and contact customisation on vs off "
                        "(prototype plugin)",
               rule="Joint wheel loads within 10 % of contact loads on a 15 deg slope; pick the cheaper source",
               fallback="Keep contact; accept the cost against the floor"),
}

gates = {}

# G1 ---------------------------------------------------------------------------------------------
g1 = load("g1")
gates["G1"] = dict(
    script="make_tree.py g1_lidar4097 autonomy --patch (route_area_lidar_0p5m.tif, 4097 samples); derive_2049.py; "
           "g1.py 5",
    measured={k: cost(v) | {"vs_lidar_2049": v["vs_lidar_2049"]} for k, v in g1.items() if k != "gate"},
    verdict="pass" if g1["gate"]["passes"] else "fail",
    numbers=g1["gate"])

# G2 ---------------------------------------------------------------------------------------------
tests = load("g2_tests_pgs")
settle = {}
for world in ("urc_delivery", "urc_equipment_servicing", "proving_ground"):
    for solver in ("pgs", "dantzig"):
        s = load(f"g2_settle_{world}_{solver}")
        if s:
            settle[f"{world}/{solver}"] = {name: {k: v for k, v in m.items() if k != "joints"}
                                           for name, m in s["models"].items()}
still = all(settle[f"urc_delivery/pgs"][k]["max_link_move_m"] < 0.01 for k in ("toolbox", "wrench"))
gates["G2"] = dict(
    script="g2_tests.py pgs; g2.py settle <world> <solver> 60",
    measured=dict(tests_with_pgs=dict(ran=tests["ran"], failures=tests["failures"], errors=tests["errors"],
                                      which="test_urc_terrain.ProvingGroundPhysics (4) and test_urc_sim.Lander "
                                            "(the key-press test), their worlds switched to PGS"),
                  settle_0p5_to_60s=settle),
    verdict="pass" if (not tests["failures"] and not tests["errors"] and still) else "fail")

# G3 ---------------------------------------------------------------------------------------------
collide, memory = load("g3_collide"), load("g3_memory")["summary"]
gates["G3"] = dict(
    script="g3.py collide; g3.py memory 3",
    measured=dict(collision=collide, visual_memory=memory),
    verdict="pass" if collide["same_as_obj"] and all(collide[k]["rover_on_slab"] and collide[k]["ball_rolled_downhill"]
                                                   for k in ("glb", "obj")) else "fail")

# G4 ---------------------------------------------------------------------------------------------
g4 = load("g4")
feathered = g4["feather"]["feathered"] or g4["feather_with_transparency"]["feathered"]
gates["G4"] = dict(
    script="g4.py",
    measured=g4,
    verdict="pass" if g4["cutout_works"] and feathered else ("partial" if g4["cutout_works"] else "fail"))

# G5 ---------------------------------------------------------------------------------------------
g5 = load("g5_joint")
table = {}
for key, v in g5.items():
    world, variant = key.split("/")
    wall = (v["full_wall_s"] - v["startup_wall_s"]) / 19_999  # difference of the median wall times
    table.setdefault(world, {})[variant] = cost(v, "cpu_ms_per_step", "real_time_factor_cpu", "cpu_ms_per_step_runs") | {
        "vs_diffdrive_dantzig": v["vs_diffdrive_dantzig"], "wall_ms_per_step": round(wall * 1e3, 4),
        "real_time_factor_wall": round(0.001 / wall, 2)}
for world, row in table.items():
    for solver in ("pgs", "dantzig"):
        row[f"{solver}_passes_by_cpu_time"] = row[f"proto_{solver}"]["real_time_factor_cpu"] >= 1.1
        row[f"{solver}_passes_by_wall_clock"] = row[f"proto_{solver}"]["real_time_factor_wall"] >= 1.1
previous = load("prev_g5_joint")
gates["G5"] = dict(
    script="g5.py joint 5 (sensors stripped, RTF 0, 20,000 steps of simulate.DRIVE_SCHEDULE, interleaved)",
    measured=table,
    earlier_runs_at_load_6_to_14={k: dict(cpu_ms_per_step=v["cpu_ms_per_step"], rtf_cpu=v["real_time_factor_cpu"])
                                  for k, v in (previous or {}).items()},
    verdict=("pass" if all(r["pgs_passes_by_cpu_time"] for r in table.values()) else
             "fail by CPU time in " + ", ".join(w for w, r in table.items() if not r["pgs_passes_by_cpu_time"])
             + "; pass by wall clock" + ("" if all(r["pgs_passes_by_wall_clock"] for r in table.values()) else
                                         " except " + ", ".join(w for w, r in table.items()
                                                                if not r["pgs_passes_by_wall_clock"]))))

# G6 ---------------------------------------------------------------------------------------------
g6 = {}
for path in sorted(R.glob("g6_*.json")):
    d = json.loads(path.read_text())
    g6[path.stem.removeprefix("g6_")] = {k: d[k] for k in ("fps", "real_time_factor", "first_frame_s",
                                                            "lens_flare_pass_added", "lens_flare_sensor_missing",
                                                            "exit_code", "errors", "rgbd", "counts") if k in d}
previous6 = {p.stem.removeprefix("prev_g6_"): {k: json.loads(p.read_text())[k] for k in ("fps", "real_time_factor")}
             for p in sorted(R.glob("prev_g6_*.json"))}


def watched_fps(d):
    """The slower of the two watched station views (eye, fly)."""
    return min(d["fps"]["/eye_camera/image"], d["fps"]["/fly_camera/image"])


main = {k: v for k, v in g6.items() if k.endswith("_rgbd1280")}
flare = {k: v["lens_flare_pass_added"] for k, v in g6.items() if "flare" in k}
gates["G6"] = dict(
    script="run_g6.sh (g6.py <world> rgbd1280|fallback|chase_flare|rgbd1280_dd 10)",
    measured=g6,
    watched_fps_full_config={k: watched_fps(v) for k, v in main.items()},
    earlier_runs_at_load_6_to_14=previous6,
    verdict="pass" if len(main) == 2 and all(watched_fps(v) >= 15 for v in main.values()) and len(flare) == 2
    and all(flare.values()) else "fail")

# G7 ---------------------------------------------------------------------------------------------
g7, inv = load("g7_cost"), load("g7_invariants")
sheets = {w: json.loads((R / f"g7_sheet_{w}.txt").read_text()) for w in ("delivery", "astrobiology")}
loss = {w: round(g7[f"{w}/2049"]["cpu_ms_per_step"] / g7[f"{w}/1025"]["cpu_ms_per_step"] - 1, 3)
        for w in ("urc_delivery", "urc_astrobiology")}
gates["G7"] = dict(
    script="make_tree.py g7_2049 delivery astrobiology --patch (2049 samples); g7_invariants.py; g7.py 5",
    measured=dict(generation_s=g7["generation_s"], rtf_loss=loss,
                  cost={k: cost(v) for k, v in g7.items() if k != "generation_s"},
                  invariants=dict(ran=inv["ran"], failures=inv["failures"], errors=inv["errors"],
                                  which="test_urc_missions and the no-physics classes of test_urc_terrain on the "
                                        "2049^2 tree"),
                  largest_sheet_changes=sheets),
    verdict="pass" if (max(g7["generation_s"].values()) <= 120 and max(loss.values()) <= 0.10
                       and not inv["failures"] and not inv["errors"]) else "fail")

# G8 ---------------------------------------------------------------------------------------------
loads = {}
for case in ("up", "across", "drive"):
    for solver in ("dantzig", "pgs"):
        d = load(f"g8_loads_{case}_{solver}")
        loads[f"{case}/{solver}"] = dict(ratio_joint_over_contact={w: v["ratio"] for w, v in d["wheels"].items()},
                                         contact_N={w: v["contact_N"] for w, v in d["wheels"].items()},
                                         sum_contact_N=d["sum_contact_N"], sum_joint_N=d["sum_joint_N"],
                                         weight_normal_N=d["rover_weight_normal_N"])
worst = max(abs(r - 1) for v in loads.values() for r in v["ratio_joint_over_contact"].values())
ab = load("g8_ab")
gates["G8"] = dict(
    script="g8_loads.py up|across|drive dantzig|pgs 1; g8_cost.py 5; g8_ab.py 10",
    measured=dict(loads_15deg=loads, worst_joint_vs_contact=round(worst, 4),
                  cost_vs_diffdrive_delivery={k: cost(v) | {"vs_diffdrive": v["vs_diffdrive"]}
                                              for k, v in load("g8_cost").items()},
                  joint_vs_contact_pairs=ab["joint_over_contact"]),
    verdict="pass" if worst <= 0.10 else "fail")

DECISIONS = {
    "G1": dict(
        decision="Autonomy at 4097^2 for collision and visual (one grid, D6).",
        notes=["RTF loss +3.1 % against its 2049^2 twin (same models and clutter), start-up 3.2 s, peak 606 MB: "
               "the loss is a ratio of interleaved runs, valid at load 5.5-11.5; start-up and memory are 9x and 6x "
               "inside their limits.",
               "The 2049^2 twin is the 4097^2 heightmap subsampled and renormalised (derive_2049.py), because "
               "generating the lidar world at 2049^2 failed: autonomy.py's routes.easy_route found no route onto "
               "the butte within EASY_ROUTE_MAX_SLOPE 16 deg (TypeError on None, g1_gen_2049.log), while at 4097^2 "
               "it found one. WS-T2 must check the easy route and its slope limit on the lidar.",
               "Today's 3DEP world (2049^2): 0.354 ms/step in the same runs."]),
    "G2": dict(
        decision="PGS is allowed in Delivery, Equipment Servicing and the proving ground as far as stability goes; "
                 "the per-world solver also needs G5's cost.",
        notes=["The toolbox and the wrench do not move (0.0 mm) from 0.5 s to 60 s with either solver.",
               "With PGS the four proving-ground physics tests and the lander key-press test pass.",
               "PGS lets loose parts creep a little more (Equipment Servicing: key 1.4 mm and 0.4 deg, fuel-tank "
               "joints 0.018 rad against 0.0005 with Dantzig): solver noise, not realism (D4)."]),
    "G3": dict(
        decision="GLB for collisions and visuals (no OBJ fallback needed).",
        notes=["The rover rests at z = 0.300 m on a 0.3 m GLB slab as on the OBJ one, and a ball rolls the same "
               "way down a 20 deg GLB and OBJ wedge (to 1 mm).",
               "1.64 M visual triangles in 16 chunks, all in view of a watched 1280 x 720 camera: peak 512 MB with "
               "GLB, 801 MB with OBJ, 314 MB without them, so GLB needs 0.41x the memory of OBJ (198 vs 487 MB); "
               "files 37.5 vs 110 MB, server wall time 3.8 vs 5.5 s."]),
    "G4": dict(
        decision="Shrub cards may use alpha cut-outs (alpha-tested at 0.5). Soft-edged decals are not available: "
                 "the spec's fallback for them applies (no detail decals), or decals whose edge is a hard, "
                 "irregular (noise-cut) alpha-test edge.",
        notes=["Cut-out: the transparent texels of an albedo map vanish (the wall behind shows through), with no "
               "<transparency> needed: gz-sim gives every albedo map an alpha test at 0.5.",
               "Feathering: an alpha ramp 0 -> 1 renders as a hard edge where alpha = 0.5. With <transparency> "
               "0.001 the material blends, so opacity follows the texture alpha from 1 down to 0.5, but the alpha "
               "test still removes everything below 0.5: the edge steps from 50 % opacity to nothing.",
               "Not measured: sorting and depth of blended (transparent-queue) materials over the Terra terrain."]),
    "G5": dict(
        decision="PGS in Delivery, the proving ground, Astrobiology, Autonomy (and rover_test); Dantzig in "
                 "Equipment Servicing (the user's Q2 answer: Dantzig with approximate per-wheel friction, no lander "
                 "optimisation).",
        notes=["Measured on an otherwise idle machine: load 1.6-5.5, of which the server under test adds about "
               "1.3 (it runs 1.3 cores); run spread 1-3 %.",
               "By the spec's CPU-time method the prototype with PGS reaches 1.29x real time in Delivery (Dantzig "
               "1.61x), 4.8x on the proving ground, 3.7x in Astrobiology and 3.4x in Autonomy (3DEP 2049^2; G1 "
               "adds 3 % for the lidar), but only 0.86x in Equipment Servicing (Dantzig 0.97x, today's DiffDrive "
               "rover 1.07x): the lander's 101 joints dominate (spec 10.1).",
               "CPU time counts the server's helper threads (gz-transport, /clock published every step) beside the "
               "physics step: the wall-clock step is 13-29 % shorter on this machine. By wall clock Equipment "
               "Servicing reaches 1.35x with Dantzig and 1.16x with PGS, Delivery 1.68x with PGS; G6 ran "
               "Equipment Servicing at RTF 0.985 of a requested 1.0 while rendering four cameras. "
               "simulate.Cost now reports both (wall_real_time_factor); sim-perf judges the wall clock.",
               "Drivetrain cost (prototype, joint loads, against DiffDrive, both Dantzig): +4.7 % to +10.8 % in "
               "every world, inside the +25 % budget. PGS over Dantzig with the prototype: +25 % in Delivery, "
               "+12 % in Equipment Servicing, 0-4 % elsewhere.",
               "Earlier runs of this gate at load 6-14 (earlier_runs_at_load_6_to_14) read up to 60 % higher and "
               "are superseded."]),
    "G6": dict(
        decision="Full config as planned: fly camera 1280 x 720, rover RGB-D 1280 x 720, shrub and pebble budgets "
                 "of the spec, lens flare allowed. Re-run this gate in WS-V with the real assets.",
        notes=["Eye and fly views 18.5 fps in both worlds (RTF 0.985-0.99 of a requested 1.0), the rover's RGB-D "
               "subscribed at 14 Hz of its 15 Hz; with the chase camera and lens flare as well 17.0-17.7 fps.",
               "The frame rate is set by rendering, not physics: the same scene with today's DiffDrive rover gives "
               "the same 18.4-18.5 fps, and the fallback (half the shrubs, no pebbles, fly 960 x 540) 18.3-18.5 fps.",
               "LensFlare logged 'Render pass added' in both worlds; its PostRender connection is reset right after "
               "(gz-sim 8 LensFlare.cc), so it costs nothing per frame afterwards. Its first message prints the "
               "camera name before it is set, hence 'named []'. The earlier run that missed the message lost the "
               "server's buffered output (killed after 15 s); the server now writes to a pseudo-terminal.",
               "Stand-ins, not the real config: 1.47 M shrub and 0.8 M pebble triangles (merged GLB chunks), a "
               "0.72 M-triangle far-field grid, a 4096^2 colour map as terrain layer 0, 80 km far clip. Not "
               "included: the patched terrain shader, sky and haze (spec 10.2: +0.7 to +4.7 ms per 1280 x 720 "
               "frame), real textures, dust. 18.5 fps leaves 3.5 fps of margin above the 15 fps floor."]),
    "G7": dict(
        decision="Delivery and Astrobiology at 2049^2.",
        notes=["Generation 6.6 s (Delivery) and 4.5 s (Astrobiology), far under the 2 min limit.",
               "No measurable RTF loss: CPU time per step +0.2 % (Delivery) and -0.7 % (Astrobiology), run spread "
               "about 2 %, load 2.4-4.9. (An earlier pair of runs at load 8-12 read -17 % and -2 %: noise.)",
               "The mission and terrain tests without physics (55) pass on the 2049^2 tree; the largest sheet "
               "changes are 1.2 cm in a zone centre height, 1.4 cm in z_max and 1 cm in an altitude."]),
    "G8": dict(
        decision="Wheel loads from JointTransmittedWrench (load_source = joint), contact customisation on; no "
                 "ContactSensorData on the tyres.",
        notes=["Joint loads are within 0.3-2.6 % of contact loads on the 15 deg slope (parked uphill and across, "
               "driving uphill at 0.2 m/s; Dantzig and PGS). Their sum equals the weight's normal component to "
               "0.1 %; the contact sum is 2.6 % low while driving.",
               "Cost in Delivery (Dantzig) against DiffDrive: contact + customisation +7.0 %, contact without "
               "customisation -0.9 %, joint + customisation -1.2 %, joint without customisation -6.6 % (load 6-10, "
               "run spread about 10 %). Joint is cheaper than contact in 8 of 10 interleaved pairs (median ratio "
               "0.933): the ContactSensorData components, not the customisation callback, carry most of the "
               "prototype's extra cost.",
               "The joint load is a static balance of the wheel (it ignores the wheel's own acceleration). The "
               "contact point for the ground lookup comes from the customisation callback, which sees every "
               "contact, so the drivetrain needs no contact sensor data for it."]),
}

doc = dict(
    format="rover-gates/1",
    spec="docs/superpowers/specs/2026-10-06-urc-realism-design.md, section 10.4 (method: section 10.3)",
    machine="Apple M4, 16 GB, macOS 26 (Darwin 25.5); gz-sim 8.10, DART, ogre2 on Metal (pixi env)",
    scripts="sim/data/research/gates/ (build.sh builds the prototype plugins; every script writes to GATES_SCRATCH)",
    method=("CPU time per physics step: plain `gz sim -s -r --iterations N` processes, user + sys from the kernel "
            "minus a one-step start-up run, interleaved cases, median of 5 (G8 A/B: 10); the rover driven by a "
            "thread of the measuring process (simulate.twist_publisher), never by Python inside the server. "
            "Sensors stripped and real_time_factor 0 unless a gate renders. The 1-minute load average is recorded: "
            "ratios hold on a loaded machine, absolute numbers need a load average below 4 (spec 10.3)."),
    summary={k: v["verdict"] for k, v in gates.items()},
    gates={k: dict(SPEC[k], **gates[k], **DECISIONS[k]) for k in gates},
    checks=dict(
        terrain_heightmap_hh=dict(
            what="sim/plugins/terrain_heightmap.hh against urc.sheet's terrain (check_heightmap.py, HmCheck)",
            measured=load("check_terrain_heightmap")),
        byte_identical=dict(
            what="WS-0's refactors leave every generated output unchanged: sim/gen_model.py and sim/gen_worlds.py "
                 "run in `git archive` trees of the baseline (232d9c2) and of WS-0's branch, sim/data shared",
            measured=load("identical"))),
)
OUT.write_text(json.dumps(doc, indent=1, ensure_ascii=False) + "\n")
print(f"wrote {OUT}: {doc['summary']}")
