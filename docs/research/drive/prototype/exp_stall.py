from exp import *
from exp_drive import straight_report
import pg, math
REAL = dict(CIRCLE, mu_s=0.95, mu_k=0.8, v_stribeck=0.03)
sel = sys.argv[1:]
# 1) turn in place on the slab (PGS: per-wheel friction honoured) with decreasing current limits
for ilim in (20, 10, 7):
    k = f"stall_turn_I{ilim}"
    if sel and not any(s in k for s in sel): continue
    D, el = case(k, [[0, 0, 0], [1.0, 0, 1.0], [6.0, 0, 0]], 7.0, opts=dict(REAL, I_lim=ilim),
                 world=pg.world(k, 19, 20, 0.0, physics_extra=PGS))
    t = D["t"]; m = window(D, 1.0, 6.0)
    yaw = np.unwrap(D["yaw"])
    st = window(D, 3.0, 6.0)
    print(f"{k}: yaw turned {np.degrees(yaw[m][-1]-yaw[m][0]):6.1f} deg, mean wz(3-6s)={D['wz'][st].mean():.3f} std={D['wz'][st].std():.3f}  "
          f"|w| per wheel={[round(float(np.abs(D['w_'+w][st]).mean()),2) for w in W]} wstd={[round(float(D['w_'+w][st].std()),2) for w in W]} "
          f"i={[round(float(np.abs(D['i_'+w][st]).mean()),1) for w in W]} sat%={[round(100*float(np.mean(np.abs(D['i_'+w][st])>=ilim-0.01)),0) for w in W]}")
# 2) drive into the 20 cm step (east) : DD vs motor current limits
for mdl, opts in (("dd", None), ("mI20", dict(REAL, I_lim=20)), ("mI10", dict(REAL, I_lim=10))):
    k = f"stall_step20_{mdl}"
    if sel and not any(s in k for s in sel): continue
    w = pg.world(k, 40.5, 28, 0.0)
    sched = [[0, 0, 0], [1.0, 0.3, 0], [11.0, 0, 0]]
    D, el = case(k, sched, 12.0, drive="dd" if mdl == "dd" else "motor", opts=opts or {}, world=w)
    m = window(D, 1.0, 11.0)
    print(f"{k}: x end={D['x'][-1]:.2f} (step face at x~44) z end={D['z'][-1]:.3f} max pitch={np.degrees(np.abs(D['pitch'][m]).max()):.1f}deg "
          f"max|tq|={[round(float(np.nanmax(np.abs(D['tq_'+w][m]))),1) for w in W]} "
          f"maxI={[round(float(np.abs(D['i_'+w][m]).max()),1) for w in W] if 'i_fl' in D else '-'} "
          f"min wheel speed (after 3 s)={[round(float(D['w_'+w][window(D,3,11)].min()),2) for w in W]}")
