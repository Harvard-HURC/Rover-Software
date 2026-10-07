from exp import *
SCHED = [[0, 0, 0], [1.0, 0, 1.0], [5.0, 0, -1.0], [7.0, 0, 0]]
cases = {
  "m_box": dict(),
  "m_circle08": dict(CIRCLE, mu_s=0.8, mu_k=0.8),
  "m_circle_strib": dict(CIRCLE, mu_s=1.0, mu_k=0.75, v_stribeck=0.02),
}
sel = sys.argv[1:] or list(cases)
res = {}
for k in sel:
    D, el = case(k, SCHED, 8.0, opts=cases[k])
    print(f"--- {k} ({el:.0f}s wall)")
    r = turn_report(D, 2.0, 5.0, 1.0, "  steady+1:")
    turn_report(D, 5.5, 7.0, -1.0, "  steady-1:")
    print("  rise90(+1)=%.3f  rise90(-1)=%.3f" % (rise(D, 1.0, r["ratio"]), rise(D, 5.0, -r["ratio"])))
