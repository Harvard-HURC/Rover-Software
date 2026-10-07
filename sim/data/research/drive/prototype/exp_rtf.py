from exp import *
import time
REAL = dict(CIRCLE, mu_s=1.0, mu_k=0.8, v_stribeck=0.03, mu_noise=0.2, mu_noise_scale=0.3)
TIRE = dict(k_lat=40000.0, c_lat=40.0, k_rad=60000.0, c_rad=170.0)
SCHED = [[0, 0, 0], [1.0, 0.8, 0], [8.0, 0, 1.0], [12.0, 0.8, 0.2], [18.0, 0, -1.0], [22.0, 0, 0]]
T = 24.0
sel = sys.argv[1:]
for k, drv, o, tire, w in (("rtf_dd", "dd", {}, None, "urc_delivery_nosensors"),
                           ("rtf_full_dantzig", "motor", REAL, TIRE, "urc_delivery_nosensors"),
                           ("rtf_full_pgs", "motor", REAL, TIRE, "urc_delivery_nosensors_pgs")):
    if sel and not any(s in k for s in sel): continue
    D, el = case(k, SCHED, T, drive=drv, opts=o, tire=tire, world=str(HERE / "worlds" / f"{w}.sdf"))
    m1 = window(D, 3, 8); m2 = window(D, 9, 12); m3 = window(D, 19, 22)
    print(f"{k:18s} wall={el:.1f}s for {T}s sim (incl. ~2 s startup) -> RTF~{T/(el-2):.1f}  "
          f"straight v={D['vbx'][m1].mean():.3f}/0.8  turn wz={D['wz'][m2].mean():.3f}/1.0 (std {D['wz'][m2].std():.3f})  turn2 wz={D['wz'][m3].mean():.3f}/-1.0  "
          f"az_rms={band_rms(D['abz'][m1],1000,2,200):.3f} ncust={D['ncust'][-1]:.0f}")
