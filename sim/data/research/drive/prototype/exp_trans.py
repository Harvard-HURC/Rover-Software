from exp import *
SCHED = [[0, 0, 0], [1.0, 0, 0.5], [3.0, 0, 0]]
TIRE = dict(k_lat=40000.0, c_lat=40.0)
STRIB = dict(CIRCLE, mu_s=1.0, mu_k=0.75, v_stribeck=0.02)
cases = {
  "tr_dd": ("dd", {}, None),
  "tr_m_box": ("motor", {}, None),
  "tr_m_circle_strib": ("motor", STRIB, None),
  "tr_m_circle_strib_tire": ("motor", STRIB, TIRE),
  "tr_m_circle_strib_tire_noramp": ("motor", dict(STRIB, accel=0), TIRE),
}
sel = sys.argv[1:] or list(cases)
for k in sel:
    drv, o, tire = cases[k]
    D, el = case(k, SCHED, 4.0, drive=drv, opts=o, tire=tire)
    t = D["t"]; wz = D["wz"]; yaw = np.unwrap(D["yaw"])
    m = (t > 1.0) & (t < 3.0)
    ss = wz[(t > 2.5) & (t < 3.0)].mean()
    t_start = t[m][np.argmax(np.abs(wz[m]) > 0.05)] - 1.0
    pk = wz[m].max()
    after = (t > 3.0)
    y3 = yaw[np.searchsorted(t, 3.0)]
    yend = yaw[-1]
    ystop = yaw[after][np.argmax(np.abs(wz[after]) < 0.005)] if np.any(np.abs(wz[after]) < 0.005) else np.nan
    t_stop = t[after][np.argmax(np.abs(wz[after]) < 0.01)] - 3.0
    # yaw-rate oscillation after stop: min (negative = spring back)
    print(f"{k:32s} delay(>0.05)={t_start*1000:5.0f}ms ss={ss:.3f} peak={pk:.3f} overshoot={100*(pk/ss-1) if ss else 0:5.1f}% "
          f"stop_time={t_stop*1000:5.0f}ms coast={np.degrees(yend-y3):6.2f}deg min_wz_after={wz[after].min():.3f} "
          f"max|i|={max(np.abs(D['i_'+w]).max() for w in W) if 'i_fl' in D else float('nan'):.1f}A max|tq|={max(np.nanmax(np.abs(D['tq_'+w])) for w in W):.1f}")
    np.savez(HERE/"logs"/f"{k}_trace.npz", t=t, wz=wz, yaw=yaw)
