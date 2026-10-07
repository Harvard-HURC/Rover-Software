from exp import *
import pg
TURN = [[0, 0, 0], [1.5, 0, 1.0], [5.5, 0, 0]]
# lanes are box tiles with SDF mu (current repo approach); sand/clay tiles too
spots = {"lane020": (18, -46), "lane035": (18, -39), "lane050": (18, -32), "lane070": (18, -25), "lane095": (18, -18),
         "sand": (19, 0), "clay": (19, 10)}
MU = {"lane020": 0.2, "lane035": 0.35, "lane050": 0.5, "lane070": 0.7, "lane095": 0.95, "sand": 0.4, "clay": 0.25}
sel = sys.argv[1:]
for spot, (x, y) in spots.items():
    for mdl in ("dd", "mc"):
        k = f"pgl_{spot}_{mdl}"
        if sel and not any(s in k for s in sel):
            continue
        if mdl == "dd":
            D, el = case(k, TURN, 6.0, drive="dd", world=pg.world(k, x, y, 0.0))
        else:  # motor + friction circle, mu from the zone (here: same value as the tile)
            mu = MU[spot]
            D, el = case(k, TURN, 6.0, drive="motor", opts=dict(CIRCLE, mu_s=mu * 1.15, mu_k=mu, v_stribeck=0.03), world=pg.world(k, x, y, 0.0))
        m = window(D, 2.5, 5.5)
        s = summary_turn(D, 2.5, 5.5, 1.0)
        print(f"{k:18s} mu={MU[spot]:.2f} ratio={s['ratio']:.3f} tq={[round(s['tq_'+w],1) for w in W]} "
              f"i={[round(float(np.mean(np.abs(D['i_'+w][m]))),1) for w in W] if 'i_fl' in D else '-'} drift={s['drift']:.3f}")
