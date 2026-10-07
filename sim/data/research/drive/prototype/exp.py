import sys, time, json
from run import *
from variants import build
from ana import *
PGS = "<dart><solver><solver_type>pgs</solver_type></solver></dart>"
CIRCLE = dict(contact=True, friction_circle=True, perp_ratio=0.0)

def case(name, schedule, seconds, drive="motor", opts=None, world=None, params=None, tire=None):
    log = str(HERE / "logs" / f"{name}.bin")
    o = dict(log=log); o.update(opts or {})
    if drive == "motor":
        md = build(name, params=params, drive="motor", plugin_opts=o, keep_diffdrive=False, effort=1000, tire=tire)
        mode = "motor_cpp"
    else:
        md = build(name, params=params, drive="probe", plugin_opts=o, tire=tire)
        mode = "diffdrive"
    w = world or flat_world("flat")
    t = time.time()
    run(name, w, schedule, seconds, mode=mode, models=[md, str(HERE / "terrain_models")], plugins=[str(HERE / "cpp/build")])
    return load(name), time.time() - t

def turn_report(D, t0, t1, wz, label):
    s = summary_turn(D, t0, t1, wz)
    m = window(D, t0, t1)
    wzs = D["wz"][m]
    r = dict(ratio=s["ratio"], std=s["std_wz"], p2p=s["p2p_wz"], drift=s["drift"],
             band_0_5=band_rms(wzs, 1000, 0.2, 5), band_5_50=band_rms(wzs, 1000, 5, 50), band_50_500=band_rms(wzs, 1000, 50, 500),
             tq=[round(float(np.nanmean(np.abs(D[f"tq_{w}"][m]))), 2) for w in W],
             tqstd=[round(float(np.nanstd(D[f"tq_{w}"][m])), 2) for w in W],
             i=[round(float(np.mean(np.abs(D[f"i_{w}"][m]))), 1) for w in W] if "i_fl" in D else None,
             wspd=[round(float(D[f"w_{w}"][m].mean()), 2) for w in W],
             wstd=[round(float(D[f"w_{w}"][m].std()), 3) for w in W],
             az_rms=band_rms(D["abz"][m], 1000, 2, 200), ax_rms=band_rms(D["abx"][m], 1000, 2, 200), ay_rms=band_rms(D["aby"][m], 1000, 2, 200))
    print(label, " ".join(f"{k}={v:.4f}" if isinstance(v, float) else f"{k}={v}" for k, v in r.items()))
    return r

def rise(D, t0, target, frac=0.9):
    m = D["t"] >= t0
    wz = D["wz"][m]; t = D["t"][m]
    k = np.argmax(np.sign(target) * wz >= frac * abs(target))
    return t[k] - t0 if k > 0 else float("nan")
