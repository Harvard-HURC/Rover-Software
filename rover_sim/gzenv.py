#!/usr/bin/env python3
"""The Gazebo environment, in one place: sim/run.sh (`eval "$(python
sim/gzenv.py)"`), the driver station, the tests (tests/simulate.py) and the
tools start Gazebo with environment().

- GZ_SIM_RESOURCE_PATH: sim/models first (model://rover, the cameras, urc_*).
- GZ_SIM_SYSTEM_PLUGIN_PATH: the plugin build directory first (sim/build, or
  another CMake build directory).
- OGRE2_RESOURCE_PATH, OGRE_RESOURCE_PATH: the env's own OGRE plugin
  directories. The conda gz-rendering has a space-padded OGRE plugin path
  baked in; without these, cameras (and on some machines the GUI) cannot
  render. A value already set wins.
- GZ_RENDERING_RESOURCE_PATH: the patched gz-rendering media in the build
  directory (MEDIA_DIR), only once the tool that makes it has marked it
  complete (MEDIA_COMPLETE); otherwise Gazebo uses its stock media.
- GZ_PARTITION, GZ_IP: only when asked (the tests run in a private partition
  on the loopback interface).
"""
import argparse
import os
import shlex
import sys
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
BUILD_DIR = SIM_DIR / "build"
MEDIA_DIR = "gz-rendering-media"  # in the build directory
MEDIA_COMPLETE = ".complete"  # in MEDIA_DIR, written last by the tool that patches the media
VARIABLES = ("GZ_SIM_RESOURCE_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH", "OGRE2_RESOURCE_PATH", "OGRE_RESOURCE_PATH",
             "GZ_RENDERING_RESOURCE_PATH", "GZ_PARTITION", "GZ_IP")


def environment(build_dir=None, *, partition=None, ip=None, base=None):
    """A copy of `base` (default os.environ) with the Gazebo variables set.
    build_dir: where the plugins are built (default sim/build); partition,
    ip: GZ_PARTITION and GZ_IP, left as they are when None."""
    env = dict(os.environ if base is None else base)
    build = Path(build_dir).resolve() if build_dir else BUILD_DIR
    for key, first in (("GZ_SIM_RESOURCE_PATH", SIM_DIR / "models"), ("GZ_SIM_SYSTEM_PLUGIN_PATH", build)):
        rest = [p for p in env.get(key, "").split(os.pathsep) if p and p != str(first)]
        env[key] = os.pathsep.join([str(first), *rest])
    prefix = Path(env.get("CONDA_PREFIX") or sys.prefix)
    env.setdefault("OGRE2_RESOURCE_PATH", str(prefix / "lib" / "OGRE-Next"))
    env.setdefault("OGRE_RESOURCE_PATH", str(prefix / "lib" / "OGRE"))
    media = build / MEDIA_DIR
    if (media / MEDIA_COMPLETE).is_file():
        env["GZ_RENDERING_RESOURCE_PATH"] = str(media)
    if partition is not None:
        env["GZ_PARTITION"] = partition
    if ip is not None:
        env["GZ_IP"] = ip
    return env


def main():
    parser = argparse.ArgumentParser(description="Print the Gazebo environment as shell exports.")
    parser.add_argument("--build-dir", help="plugin build directory (default sim/build)")
    args = parser.parse_args()
    env = environment(args.build_dir)
    for key in VARIABLES:
        if key in env:
            print(f"export {key}={shlex.quote(env[key])}")


if __name__ == "__main__":
    main()
