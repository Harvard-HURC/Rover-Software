import numpy as np, json
from pathlib import Path
HERE = Path(__file__).resolve().parent
W = ("fl", "rl", "fr", "rr")
CCOLS = ["tq", "app", "i", "d", "w_o", "sp", "fn", "ft", "nc", "rr"]

def load(name):
    z = np.load(HERE / "logs" / f"{name}.npz", allow_pickle=True)
    D = {c: z["data"][:, k] for k, c in enumerate(z["cols"])}
    D["cfg"] = json.loads(str(z["cfg"]))
    cl = HERE / "logs" / f"{name}.bin"
    if cl.exists():
        a = np.fromfile(cl, dtype=np.float64)
        n = 1 + 4 * len(CCOLS) + 1
        a = a[: len(a) // n * n].reshape(-1, n)
        m = min(len(a), len(D["t"]))
        for k in list(D):
            if k != "cfg":
                D[k] = D[k][:m]
        D["ct"] = a[:m, 0]
        for j, w in enumerate(W):
            for c, col in enumerate(CCOLS):
                D[f"{col}_{w}"] = a[:m, 1 + j * len(CCOLS) + c]
        D["ncust"] = a[:m, -1]
    return D

def window(D, t0, t1):
    return (D["t"] >= t0) & (D["t"] < t1)

def unwrap_yaw(D):
    return np.unwrap(D["yaw"])

def summary_turn(D, t0, t1, wz_cmd, label=""):
    m = window(D, t0, t1)
    wz = D["wz"][m]
    yaw = unwrap_yaw(D)
    dyaw = yaw[m][-1] - yaw[m][0]
    T = t1 - t0
    # ripple: std of yaw rate after 10 Hz?  report raw std and p2p
    out = dict(label=label, mean_wz=wz.mean(), ratio=wz.mean() / wz_cmd if wz_cmd else np.nan,
               std_wz=wz.std(), p2p_wz=np.ptp(wz), drift=float(np.hypot(D["x"][m][-1]-D["x"][m][0], D["y"][m][-1]-D["y"][m][0])))
    for w in W:
        out[f"w_{w}"] = D[f"w_{w}"][m].mean()
        if f"tq_{w}" in D:
            out[f"tq_{w}"] = np.nanmean(np.abs(D[f"tq_{w}"][m]))
            out[f"tqstd_{w}"] = np.nanstd(D[f"tq_{w}"][m])
    if "fn_fl" in D:
        out["fn"] = [float(D[f"fn_{w}"][m].mean()) for w in W]
    return out

def fmt(d):
    return " ".join(f"{k}={v:.3f}" if isinstance(v, float) else f"{k}={v}" for k, v in d.items())

def spectrum(x, fs=1000.0):
    x = x - x.mean()
    f = np.fft.rfftfreq(len(x), 1 / fs)
    P = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
    return f, P

def band_rms(x, fs, lo, hi):
    X = np.fft.rfft(x - x.mean())
    f = np.fft.rfftfreq(len(x), 1 / fs)
    X[(f < lo) | (f >= hi)] = 0
    return float(np.sqrt(np.mean(np.fft.irfft(X, len(x)) ** 2)))
