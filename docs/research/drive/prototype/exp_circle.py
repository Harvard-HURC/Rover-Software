from exp_baseline import *
cases = {
  "box_mu1": dict(contact=True, friction_circle=False, mu_s=1.0, mu_k=1.0),
  "circle_mu1_p1": dict(contact=True, friction_circle=True, mu_s=1.0, mu_k=1.0, perp_ratio=1.0),
  "circle_mu1_p03": dict(contact=True, friction_circle=True, mu_s=1.0, mu_k=1.0, perp_ratio=0.3),
  "circle_mu1_p01": dict(contact=True, friction_circle=True, mu_s=1.0, mu_k=1.0, perp_ratio=0.1),
  "circle_mu05_p01": dict(contact=True, friction_circle=True, mu_s=0.5, mu_k=0.5, perp_ratio=0.1),
}
import sys
sel = sys.argv[1:] or list(cases)
for k in sel:
    n = f"dd_turn_{k}"
    el = dd(n, [[0, 0, 0], [1.0, 0, 1.0]], 5.0, opts=cases[k])
    D = load(n)
    s = summary_turn(D, 2.0, 5.0, 1.0)
    print(f"{k} ({el:.0f}s): ratio={s['ratio']:.3f} std={s['std_wz']:.4f} drift={s['drift']:.3f} tq={[round(s[f'tq_{w}'],2) for w in W]} fn={[round(x) for x in s['fn']]} ncust={D['ncust'][-1]:.0f}")
