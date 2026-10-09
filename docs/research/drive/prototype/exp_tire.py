from exp import *
SCHED = [[0, 0, 0], [1.0, 0, 1.0], [5.0, 0, 0.3], [8.0, 0, 0]]
TIRE = dict(k_lat=40000.0, c_lat=40.0)
cases = {
  "t_circle_strib": (dict(CIRCLE, mu_s=1.0, mu_k=0.75, v_stribeck=0.02), TIRE),
  "t_circle_nostrib": (dict(CIRCLE, mu_s=0.8, mu_k=0.8), TIRE),
  "nt_circle_strib": (dict(CIRCLE, mu_s=1.0, mu_k=0.75, v_stribeck=0.02), None),
}
sel = sys.argv[1:] or list(cases)
for k in sel:
    o, tire = cases[k]
    D, el = case(k, SCHED, 8.0, opts=o, tire=tire)
    print(f"--- {k} ({el:.0f}s wall)")
    turn_report(D, 2.0, 5.0, 1.0, "  wz=1.0:")
    turn_report(D, 6.0, 8.0, 0.3, "  wz=0.3:")
