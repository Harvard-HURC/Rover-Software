"""G7: what changes in a mission sheet between two generations (scratch)."""
import json
import sys
from pathlib import Path


def walk(a, b, path, out):
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b)):
            if k not in a or k not in b:
                out.setdefault("missing", []).append(f"{path}/{k}")
            else:
                walk(a[k], b[k], f"{path}/{k}", out)
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            out.setdefault("length", []).append(f"{path}: {len(a)} -> {len(b)}")
        for k, (x, y) in enumerate(zip(a, b)):
            walk(x, y, f"{path}[{k}]", out)
    elif isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
        if a != b:
            key = path.split("/")[-1].split("[")[0]
            d = abs(a - b)
            entry = out.setdefault("numbers", {}).setdefault(key, [0.0, ""])
            if d > entry[0]:
                out["numbers"][key] = [round(d, 4), f"{path}: {a} -> {b}"]
    elif a != b:
        out.setdefault("other", []).append(f"{path}: {str(a)[:60]} -> {str(b)[:60]}")


a, b = (json.loads(Path(p).read_text()) for p in sys.argv[1:3])
out = {}
walk(a, b, "", out)
print(json.dumps(out, indent=1))
