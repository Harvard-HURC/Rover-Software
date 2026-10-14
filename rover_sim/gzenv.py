#!/usr/bin/env python3
"""The Gazebo environment, in one place: rover_sim/run.sh (`eval "$(python
rover_sim/gzenv.py)"`), the driver station, the tests (tests/simulate.py) and
the tools start Gazebo with environment().

- GZ_SIM_RESOURCE_PATH: rover_sim/models first (model://rover, the cameras,
  urc_*).
- GZ_SIM_SYSTEM_PLUGIN_PATH: the plugin build directory first: build/rover_sim
  in the colcon workspace whose src/ this repository is, where colcon build
  puts the plugins (BUILD_DIR), or another CMake build directory.
- OGRE2_RESOURCE_PATH, OGRE_RESOURCE_PATH: the env's own OGRE plugin
  directories. The conda gz-rendering has a space-padded OGRE plugin path
  baked in; without these, cameras (and on some machines the GUI) cannot
  render. A value already set wins.
- GZ_RENDERING_RESOURCE_PATH: the patched gz-rendering media in the build
  directory (MEDIA_DIR), only once the tool that makes it has marked it
  complete (MEDIA_COMPLETE); otherwise Gazebo uses its stock media.
- GZ_SIM_SYSTEM_PLUGIN_PATH, last: the environment's lib, where
  gz_ros2_control's system plugin is (ros_gz_sim's own launch file adds
  LD_LIBRARY_PATH the same way); Gazebo does not search it.
- AMENT_PREFIX_PATH: the workspace's install/rover_sim first, once colcon has
  installed it (INSTALL_DIR): pluginlib finds RoverSimSystem there, which
  gz_ros2_control loads, without a shell that sourced install/.
- GZ_PARTITION, GZ_IP: only when asked (the tests run in a private partition
  on the loopback interface); ROS_DOMAIN_ID and
  ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST: only when asked (the tests and the
  launcher's worlds take private_ros_domain()).

private_ros_home() gives a ROS 2 run a ROS_HOME of its own for its lock files
(the controller spawner's: sim.launch.py, the tests).
"""
import argparse
import os
import shlex
import sys
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
WORKSPACE = SIM_DIR.parents[1]  # the colcon workspace: this repository is its src/
BUILD_DIR = WORKSPACE / "build" / "rover_sim"  # where colcon builds this package
INSTALL_DIR = WORKSPACE / "install" / "rover_sim"  # where colcon installs it: RoverSimSystem, pluginlib's index
MEDIA_DIR = "gz-rendering-media"  # in the build directory
MEDIA_COMPLETE = ".complete"  # in MEDIA_DIR, written last by the tool that patches the media
VARIABLES = ("GZ_SIM_RESOURCE_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH", "OGRE2_RESOURCE_PATH", "OGRE_RESOURCE_PATH",
             "GZ_RENDERING_RESOURCE_PATH", "AMENT_PREFIX_PATH", "GZ_PARTITION", "GZ_IP", "ROS_DOMAIN_ID",
             "ROS_AUTOMATIC_DISCOVERY_RANGE")


def private_ros_domain(seed):
    """A ROS_DOMAIN_ID for runs that must not meet another ROS 2 graph on this machine (the tests, the launcher's
    worlds), from `seed` (a process id): 50-99, inside the 0-101 that keeps DDS's ports below the ephemeral ranges of
    Linux, macOS and Windows."""
    return 50 + seed % 50


def private_ros_home(directory, base=None):
    """ROS_HOME and ROS_LOG_DIR for a ROS 2 run whose lock files must not be another run's: ROS_HOME `directory`, and
    ROS_LOG_DIR where the logs went before (`base`'s, default os.environ: its ROS_LOG_DIR, else its ROS_HOME's log,
    else ~/.ros/log), so that only the locks move. ros2_control's spawner holds a lock in $ROS_HOME/locks while it
    waits for its controller manager, the same file for every spawner on the machine whatever its ROS domain, and
    gives up after some 115 s: one stack loading a big world would make another's spawner fail."""
    env = os.environ if base is None else base
    logs = env.get("ROS_LOG_DIR") or str(Path(env.get("ROS_HOME") or Path.home() / ".ros").expanduser() / "log")
    return {"ROS_HOME": str(directory), "ROS_LOG_DIR": logs}


def environment(build_dir=None, *, partition=None, ip=None, base=None, ros_domain=None, install_dir=None):
    """A copy of `base` (default os.environ) with the Gazebo variables set.
    build_dir: where the plugins are built (default BUILD_DIR); install_dir:
    where colcon installed this package (default INSTALL_DIR); partition, ip:
    GZ_PARTITION and GZ_IP, ros_domain: ROS_DOMAIN_ID (with LOCALHOST
    discovery), left as they are when None."""
    env = dict(os.environ if base is None else base)
    build = Path(build_dir).resolve() if build_dir else BUILD_DIR
    for key, first in (("GZ_SIM_RESOURCE_PATH", SIM_DIR / "models"), ("GZ_SIM_SYSTEM_PLUGIN_PATH", build)):
        rest = [p for p in env.get(key, "").split(os.pathsep) if p and p != str(first)]
        env[key] = os.pathsep.join([str(first), *rest])
    prefix = Path(env.get("CONDA_PREFIX") or sys.prefix)
    plugins = env["GZ_SIM_SYSTEM_PLUGIN_PATH"].split(os.pathsep)
    if str(prefix / "lib") not in plugins:
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join([*plugins, str(prefix / "lib")])
    install = Path(install_dir).resolve() if install_dir else INSTALL_DIR
    if install.is_dir():
        rest = [p for p in env.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if p and p != str(install)]
        env["AMENT_PREFIX_PATH"] = os.pathsep.join([str(install), *rest])
    env.setdefault("OGRE2_RESOURCE_PATH", str(prefix / "lib" / "OGRE-Next"))
    env.setdefault("OGRE_RESOURCE_PATH", str(prefix / "lib" / "OGRE"))
    media = build / MEDIA_DIR
    if (media / MEDIA_COMPLETE).is_file():
        env["GZ_RENDERING_RESOURCE_PATH"] = str(media)
    if partition is not None:
        env["GZ_PARTITION"] = partition
    if ip is not None:
        env["GZ_IP"] = ip
    if ros_domain is not None:
        env["ROS_DOMAIN_ID"] = str(ros_domain)
        env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    return env


def main():
    parser = argparse.ArgumentParser(description="Print the Gazebo environment as shell exports.")
    parser.add_argument("--build-dir", help="plugin build directory (default: the workspace's build/rover_sim)")
    args = parser.parse_args()
    env = environment(args.build_dir)
    for key in VARIABLES:
        if key in env:
            print(f"export {key}={shlex.quote(env[key])}")


if __name__ == "__main__":
    main()
