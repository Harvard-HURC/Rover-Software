from exp import *
import pg, math
REAL = dict(CIRCLE, mu_s=0.95, mu_k=0.8, v_stribeck=0.03)
def straight_report(D, t0, t1, v, label):
    m = window(D, t0, t1)
    vx = D["vbx"][m]
    r = dict(v=float(vx.mean()), v_ratio=float(vx.mean() / v), v_std=float(vx.std()),
             wz_std=float(D["wz"][m].std()),
             tq=[round(float(np.nanmean(D[f"tq_{w}"][m])), 2) for w in W],
             tqmax=[round(float(np.nanmax(np.abs(D[f"tq_{w}"][m]))), 1) for w in W],
             tqstd=[round(float(np.nanstd(D[f"tq_{w}"][m])), 2) for w in W],
             i=[round(float(np.mean(np.abs(D[f"i_{w}"][m]))), 1) for w in W] if "i_fl" in D else None,
             imax=[round(float(np.max(np.abs(D[f"i_{w}"][m]))), 1) for w in W] if "i_fl" in D else None,
             wstd=[round(float(D[f"w_{w}"][m].std()), 3) for w in W],
             az=band_rms(D["abz"][m], 1000, 2, 200), ax=band_rms(D["abx"][m], 1000, 2, 200),
             pitch_rate_rms=band_rms(D["wy"][m], 1000, 0.5, 200), roll_rate_rms=band_rms(D["wx"][m], 1000, 0.5, 200))
    print(label, " ".join(f"{k}={v:.3f}" if isinstance(v, float) else f"{k}={v}" for k, v in r.items()))
    return r
if __name__ == "__main__":
    sel = sys.argv[1:]
    # (name, spot x, y, yaw, speed, seconds, extra opts)
    runs = [
      ("flat", None, 1.0, 7.0, {}),
      ("wash", (28, 12, 0.0), 0.5, 12.0, {}),     # east across corrugations (washboard centred x=38)
      ("wash1", (28, 12, 0.0), 1.0, 9.0, {}),
      ("garden10", (34, -46, 0.0), 0.5, 12.0, {}),
      ("sand_crr", (21, 0, math.pi), 0.5, 10.0, dict(zones=[[ -4, 22, -2.5, 2.5, 0.46, 0.4, 0.20, 0.0]])),
      ("sand_crr_slip", (21, 0, math.pi), 0.5, 10.0, dict(zones=[[ -4, 22, -2.5, 2.5, 0.46, 0.4, 0.20, 1.0]])),
      ("ramp_mu035", (20, -39, math.pi), 0.5, 20.0, dict(zones=[[ -14, 22, -41.5, -36.5, 0.40, 0.35, 0.0, 0.0]])),
    ]
    for name, spot, v, T, extra in runs:
        for mdl in ("dd", "mc"):
            k = f"drv_{name}_{mdl}"
            if sel and not any(s in k for s in sel):
                continue
            w = flat_world("flat") if spot is None else pg.world(k, *spot)
            sched = [[0, 0, 0], [1.0, v, 0], [T - 1.0, 0, 0]]
            if mdl == "dd":
                D, el = case(k, sched, T, drive="dd", world=w, opts=(dict(extra) if "zones" in extra else {}))
            else:
                D, el = case(k, sched, T, drive="motor", world=w, opts=dict(REAL, **extra))
            print(f"--- {k} ({el:.0f}s wall)  dist={math.hypot(D['x'][-1]-D['x'][0], D['y'][-1]-D['y'][0]):.2f} m")
            straight_report(D, 3.0, T - 1.0, v, "  cruise:")
            # acceleration phase
            m = window(D, 1.0, 3.0); vx = D["vbx"][m]; tt = D["t"][m]
            k90 = np.argmax(vx > 0.9 * v)
            print(f"  t90={tt[k90]-1.0 if k90 else float('nan'):.3f}s  stop: v at T-0.8={D['vbx'][np.searchsorted(D['t'], T-0.8)]:.3f}")
