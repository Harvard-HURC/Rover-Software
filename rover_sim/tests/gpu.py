"""Whether this machine can render: ogre2 needs a GPU (Metal on every Mac, a
render node on Linux, WSL2's /dev/dxg). The rendering tests take
@gpu.needs_gpu and skip, saying why, where it cannot (CI's Linux runners);
ROVER_RENDERING=1 or 0 overrides the guess."""
import os
import sys
import unittest
from pathlib import Path


def rendering(environ=None, platform=None, dev=Path("/dev")):
    """(True, "") where ogre2 can render, else (False, why)."""
    environ = os.environ if environ is None else environ
    platform = sys.platform if platform is None else platform
    forced = environ.get("ROVER_RENDERING")
    if forced in ("0", "1"):
        return (True, "") if forced == "1" else (False, "ROVER_RENDERING=0")
    if platform == "darwin" or any((dev / "dri").glob("renderD*")) or (dev / "dxg").exists():
        return True, ""
    return False, f"no GPU (no {dev}/dri/renderD*, no {dev}/dxg); ROVER_RENDERING=1 runs it anyway"


AVAILABLE, WHY = rendering()
needs_gpu = unittest.skipUnless(AVAILABLE, f"needs a GPU to render: {WHY}")
