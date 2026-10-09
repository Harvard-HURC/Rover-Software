from exp import *
from exp_drive import straight_report
import pg, math
REAL = dict(CIRCLE, mu_s=0.95, mu_k=0.8, v_stribeck=0.03)
sel = sys.argv[1:]
for solver, extra in (("dantzig", ""), ("pgs", PGS)):
    # rest + turn on slab
    k = f"pgs_{solver}_slab"
    if not sel or any(s in k for s in sel):
        D, el = case(k, [[0, 0, 0], [4.0, 0, 1.0], [8.0, 0, 0]], 10.0, opts=REAL, world=pg.world(k, 19, 20, 0.0, physics_extra=extra))
        m = window(D, 1.0, 4.0)
        print(f"--- {k} ({el:.1f}s wall for 10 s) rest: drift={math.hypot(D['x'][m][-1]-D['x'][m][0], D['y'][m][-1]-D['y'][m][0])*1000:.2f} mm "
              f"z_std={D['z'][m].std()*1000:.3f} mm az_rms={band_rms(D['abz'][m],1000,1,200):.4f} wz_std={D['wz'][m].std():.5f}")
        turn_report(D, 5.0, 8.0, 1.0, "  turn:")
        mm = window(D, 5, 8)
        print("   fn", [round(float(D['fn_'+w][mm].mean())) for w in W], "ft", [round(float(D['ft_'+w][mm].mean())) for w in W])
    for spot, xy, v in (("wash", (28, 12, 0.0), 0.5), ("garden10", (34, -46, 0.0), 0.5)):
        k = f"pgs_{solver}_{spot}"
        if sel and not any(s in k for s in sel):
            continue
        D, el = case(k, [[0, 0, 0], [1.0, v, 0], [11.0, 0, 0]], 12.0, opts=REAL, world=pg.world(k, *xy, physics_extra=extra))
        print(f"--- {k} ({el:.1f}s wall)")
        straight_report(D, 3.0, 11.0, v, "  cruise:")
        k2 = k + "_turn"
        D, el = case(k2, [[0, 0, 0], [1.5, 0, 1.0], [7.5, 0, 0]], 8.0, opts=REAL, world=pg.world(k2, *(((38, 12, 0.0)) if spot == "wash" else (42, -46, 0.0)), physics_extra=extra))
        turn_report(D, 2.5, 7.5, 1.0, "  turn:")
