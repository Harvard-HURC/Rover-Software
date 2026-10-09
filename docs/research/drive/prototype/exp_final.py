from exp import *
import pg
FULL = dict(CIRCLE, mu_s=1.0, mu_k=0.8, v_stribeck=0.03, mu_noise=0.2, mu_noise_scale=0.3)
TIRE = dict(k_lat=40000.0, c_lat=40.0, k_rad=60000.0, c_rad=170.0)
sel = sys.argv[1:]
rows = []
for wz in (0.15, 0.5, 1.0):
    for k, drv, o, tire, phys in (("dd", "dd", {}, None, ""), ("full", "motor", FULL, TIRE, PGS)):
        n = f"fin_slab_{k}_{wz}"
        if sel and not any(s in n for s in sel): continue
        D, el = case(n, [[0, 0, 0], [1.0, 0, wz], [8.0, 0, 0]], 9.0, drive=drv, opts=o, tire=tire, world=pg.world(n, 19, 20, 0.0, physics_extra=phys))
        r = turn_report(D, 3.0, 8.0, wz, f"{n:22s}")
        t = D["t"]; w = D["wz"]
        ss = w[window(D, 3, 8)].mean()
        t90 = t[(t > 1.0)][np.argmax(np.abs(w[t > 1.0]) > 0.9 * abs(ss))] - 1.0
        after = t > 8.0
        print(f"      t90={t90*1000:.0f} ms, yaw-rate p2p/mean={r['p2p']/abs(ss):.2f}, stop: |wz|<0.01 after {(t[after][np.argmax(np.abs(w[after])<0.01)]-8.0)*1000:.0f} ms")
