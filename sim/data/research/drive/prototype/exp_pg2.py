from exp import *
import pg
REAL = dict(CIRCLE, mu_s=0.95, mu_k=0.8, v_stribeck=0.03)
NOISE = dict(REAL, mu_noise=0.25, mu_noise_scale=0.3)
TURN = [[0, 0, 0], [1.5, 0, 1.0], [7.5, 0, 0]]
spots = {"garden10": (42, -46, 0.0), "garden20": (42, -36, 0.0), "wash": (38, 12, 0.0), "slab": (19, 20, 0.0)}
models = {"dd": ("dd", {}), "mc": ("motor", REAL), "mcn": ("motor", NOISE)}
import itertools
sel = sys.argv[1:]
for spot, mdl in itertools.product(spots, models):
    k = f"pgt_{spot}_{mdl}"
    if sel and not any(s in k for s in sel):
        continue
    drv, o = models[mdl]
    x, y, yaw = spots[spot]
    D, el = case(k, TURN, 8.0, drive=drv, opts=o, world=pg.world(k, x, y, yaw))
    print(f"--- {k} ({el:.0f}s wall)")
    r = turn_report(D, 2.5, 7.5, 1.0, "  turn:")
    m = window(D, 2.5, 7.5)
    print("   max|tq|", [round(float(np.nanmax(np.abs(D['tq_'+w][m]))),1) for w in W], "min|w|", [round(float(np.abs(D['w_'+w][m]).min()),2) for w in W],
          "yaw turned deg", round(float(np.degrees(np.unwrap(D['yaw'])[m][-1]-np.unwrap(D['yaw'])[m][0])),1))
