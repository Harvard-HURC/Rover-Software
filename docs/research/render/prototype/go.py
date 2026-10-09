"""Run one render config in a subprocess (ogre2 starts once per process), with peak memory."""
import json
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PY = "/Users/alarion239/Desktop/Rover/.pixi/envs/default/bin/python"
VIEWS = json.loads((HERE / "views_autonomy.json").read_text())


def render(tag, world, views=None, seconds=1.0, replace=(), extra_world="", env=None, quiet=True):
    cfg = {"world": str(world), "out": str(HERE / "out" / tag), "seconds": seconds,
           "views": views if views is not None else VIEWS, "replace": list(replace), "extra_world": extra_world}
    path = HERE / f"cfg_{tag}.json"
    path.write_text(json.dumps(cfg))
    import os
    e = dict(os.environ)
    e.update(env or {})
    p = subprocess.run(["/usr/bin/time", "-l", PY, str(HERE / "render.py"), str(path)], capture_output=True,
                       text=True, cwd=HERE, env=e, timeout=1200)
    text = p.stdout + p.stderr
    m = re.search(r"RESULT (.*)", text)
    res = json.loads(m.group(1)) if m else {"error": text[-3000:]}
    fp = re.search(r"(\d+)\s+peak memory footprint", text)
    rss = re.search(r"(\d+)\s+maximum resident set size", text)
    res["peak_footprint_MB"] = round(int(fp.group(1)) / 2**20) if fp else None
    res["max_rss_MB"] = round(int(rss.group(1)) / 2**20) if rss else None
    errs = [l for l in text.splitlines() if ("[Err]" in l or "WARN" in l or "rror" in l) and "Ogre Plugin" not in l]
    res["errors"] = errs[:15]
    (HERE / "out" / f"{tag}.log").write_text(text)
    if not quiet:
        print(text[-4000:])
    return res


if __name__ == "__main__":
    print(json.dumps(render(sys.argv[1], sys.argv[2]), indent=1))
