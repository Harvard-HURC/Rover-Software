from exp import *
from exp_drive import straight_report
import pg, math
REAL = dict(CIRCLE, mu_s=0.95, mu_k=0.8, v_stribeck=0.03)
TIRE = dict(k_lat=40000.0, c_lat=40.0, k_rad=60000.0, c_rad=170.0)
sel = sys.argv[1:]
for spot, xy, v in (("wash1", (28, 12, 0.0), 1.0), ("garden10", (34, -46, 0.0), 0.5), ("garden20", (34, -36, 0.0), 0.5)):
    for mdl, tire, drv in (("dd", None, "dd"), ("mc", None, "motor"), ("mct", TIRE, "motor")):
        k = f"vib_{spot}_{mdl}"
        if sel and not any(s in k for s in sel): continue
        T = 9.0 if v == 1.0 else 12.0
        D, el = case(k, [[0, 0, 0], [1.0, v, 0], [T - 1, 0, 0]], T, drive=drv, opts=(REAL if drv == "motor" else {}), world=pg.world(k, *xy), tire=tire)
        m = window(D, 3.0, T - 1)
        az = D["abz"][m]; ax = D["abx"][m]
        f, P = spectrum(az)
        top = f[np.argsort(P)[-3:]][::-1]
        print(f"{k:20s} ({el:.0f}s) az_rms(2-200Hz)={band_rms(az,1000,2,200):.3f} az_rms(20-200Hz)={band_rms(az,1000,20,200):.3f} ax_rms={band_rms(ax,1000,2,200):.3f} "
              f"pitch_rate_rms={band_rms(D['wy'][m],1000,0.5,200):.4f} az_peak_f={np.round(top,1)} tqstd={[round(float(np.nanstd(D['tq_'+w][m])),2) for w in W]}")
