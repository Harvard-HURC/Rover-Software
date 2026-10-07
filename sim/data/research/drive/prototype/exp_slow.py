from exp import *
TIRE = dict(k_lat=40000.0, c_lat=40.0, k_rad=60000.0, c_rad=170.0)
NOSTRIB = dict(CIRCLE, mu_s=0.8, mu_k=0.8)
STRIB = dict(CIRCLE, mu_s=1.0, mu_k=0.75, v_stribeck=0.03)
sel = sys.argv[1:]
for wz in (0.1, 0.25):
    for k, o, tire, phys in (("nostrib", NOSTRIB, None, ""), ("strib", STRIB, None, ""), ("strib_tire", STRIB, TIRE, ""),
                             ("strib_tire_pgs", STRIB, TIRE, PGS), ("nostrib_tire_pgs", NOSTRIB, TIRE, PGS)):
        n = f"slow{wz}_{k}"
        if sel and not any(s in n for s in sel): continue
        D, el = case(n, [[0, 0, 0], [1.0, 0, wz], [9.0, 0, 0]], 9.5, opts=dict(o, accel=4.0), tire=tire, world=flat_world("flat_" + ("pgs" if phys else "dz"), physics_extra=phys))
        r = turn_report(D, 3.0, 9.0, wz, f"{n:26s}")
        m = window(D, 3.0, 9.0)
        f, P = spectrum(D["wz"][m]); P[f < 0.3] = 0
        print("      wz spectral peaks Hz:", np.round(f[np.argsort(P)[-3:]][::-1], 2), " frac time |wz|<0.2*mean:", round(float(np.mean(np.abs(D['wz'][m]) < 0.2 * abs(D['wz'][m].mean()))), 3))
