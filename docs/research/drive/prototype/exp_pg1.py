from exp import *
import pg
SCHED = [[0, 0, 0], [1.5, 0, 1.0], [5.5, 0, 0]]
SLAB = [0, 22, 17.5, 22.5]
cases = {
  "pg_slab_dd": ("dd", {}, (19, 20)),
  "pg_slab_m_circle_mu1": ("motor", dict(CIRCLE, mu_s=1.0, mu_k=1.0), (19, 20)),
  "pg_slab_m_circle_zone03": ("motor", dict(CIRCLE, mu_s=1.0, mu_k=1.0, zones=[SLAB + [0.3, 0.3, 0.0, 0.0]]), (19, 20)),
}
sel = sys.argv[1:] or list(cases)
for k in sel:
    drv, o, (x, y) = cases[k]
    D, el = case(k, SCHED, 6.0, drive=drv, opts=o, world=pg.world(k, x, y, 0.0))
    print(f"--- {k} ({el:.0f}s wall) ncust={D['ncust'][-1]:.0f}")
    turn_report(D, 2.5, 5.5, 1.0, "  wz=1.0:")
