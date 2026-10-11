"""Whether this machine can render: ogre2 needs a GPU (Metal on every Mac, a
render node on Linux, WSL2's /dev/dxg). The rendering tests take
@gpu.needs_gpu and skip, saying why, where it cannot (CI's Linux runners);
ROVER_RENDERING=1 or 0 overrides the guess."""
import os
import subprocess
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


def paravirtual(platform=None, read=None):
    """Whether this Mac renders on Apple's paravirtual GPU, i.e. runs as a macOS virtual machine (kern.hv_vmm_present
    is 1 under a hypervisor), as GitHub's macOS runners do. That GPU draws large merged meshes (the clutter's rocks,
    shrubs and pebbles) as spikes where a physical Mac draws them right (docs/sim/gazebo-lessons.md)."""
    platform = sys.platform if platform is None else platform
    if platform != "darwin":
        return False
    if read is None:
        def read():
            return subprocess.run(["sysctl", "-n", "kern.hv_vmm_present"], capture_output=True, text=True).stdout
    try:
        return read().strip() == "1"
    except OSError:
        return False


PARAVIRTUAL = paravirtual()
needs_physical_gpu = unittest.skipIf(
    PARAVIRTUAL, "Apple's paravirtual GPU (a macOS virtual machine, e.g. GitHub's macOS runner) draws large merged "
                 "meshes as spikes; a physical Mac renders them (docs/sim/gazebo-lessons.md)")
