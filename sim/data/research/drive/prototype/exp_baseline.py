import sys, time
from run import *
from variants import build
from ana import *

def dd(name, schedule, seconds, world=None, params=None, opts=None):
    log = str(HERE / "logs" / f"{name}.bin")
    o = dict(log=log); o.update(opts or {})
    md = build(name, params=params, drive="probe", plugin_opts=o)
    w = world or flat_world("flat")
    t = time.time()
    run(name, w, schedule, seconds, mode="diffdrive", models=[md], plugins=[str(HERE / "cpp/build")])
    return time.time() - t

if __name__ == "__main__":
    which = sys.argv[1:]
    S = []
    if "turn" in which:
        for wz in (0.5, 1.0):
            n = f"dd_turn{wz}"
            el = dd(n, [[0, 0, 0], [1.0, 0, wz]], 6.0)
            D = load(n)
            print(f"{n} ({el:.0f}s wall):", fmt(summary_turn(D, 2.0, 6.0, wz)))
    if "straight" in which:
        for v in (0.5, 1.0):
            n = f"dd_str{v}"
            el = dd(n, [[0, 0, 0], [1.0, v, 0]], 5.0)
            D = load(n)
            m = window(D, 2.0, 5.0)
            print(f"{n} ({el:.0f}s):", "vx", D["vbx"][m].mean(), "std", D["vbx"][m].std(),
                  "tq", [float(np.nanmean(D[f'tq_{w}'][m])) for w in W], "fn", [float(D[f'fn_{w}'][m].mean()) for w in W])
