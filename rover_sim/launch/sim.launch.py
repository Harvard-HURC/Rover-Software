"""
The rover in a Gazebo world with its ROS 2 stack (design spec 2026-10-09, section 6.3):

    ros2 launch rover_sim sim.launch.py world:=urc_delivery

starts
- Gazebo's server on a copy of the world in which the rover is the one with ros2_control, at the world's rover pose
  (the mission sheet's start); the worlds themselves keep model://rover, the rover of every ROS-free run (the tests,
  the referee, the maps). With gui:=true Gazebo's GUI too, as a separate process (macOS cannot run both in one);
  closing it, or the server exiting, stops everything;
- robot_state_publisher on that rover's description (urdf/rover_sim.urdf.xacro with ros2_control): /robot_description,
  /tf_static and the joints' /tf;
- the controllers of rover_control's config/controllers.yaml in the controller manager that gz_ros2_control runs
  inside Gazebo (rover_control's launch/controllers.launch.py): joint_state_broadcaster (/joint_states) and the drive
  controller (controller:=<name>), which takes /cmd_vel (geometry_msgs/TwistStamped) and publishes /odom and
  odom -> base_link. The spawner gets a ROS_HOME of this launch's own for its lock file (gzenv.private_ros_home). A
  world reset (the GUI's button, the world's control service) reloads gz_ros2_control, which does not implement
  Gazebo's reset, and its new controller manager starts without controllers: they are spawned again each time
  RoverSimSystem comes up after the first. A spawner that fails stops everything, saying so: the rover could not be
  driven;
- the bridge of the sensors and of the simulation's own topics (bridge.yaml): /imu, /gps/fix, /camera/*, /clock,
  /ground_truth/odom and more (docs/topics.md);
- with station:=true the driver station, attached to the world (pixi run drive starts this launch itself, as its
  parent: it is the station).

Arguments: world (a name in rover_sim/worlds or an .sdf path; default rover_test), gui (default true where there is
a display), station (default false), controller (default drive_controller), sensors (false: without Gazebo's Sensors
system, so nothing renders and no GPU is needed; the cameras stay dark, the IMU and GNSS work).

The simulation runs from its source tree, rover_sim/ beside rover_description/ and rover_control/, as pixi run sim
does: the generated worlds and models are there, and gzenv.py sets Gazebo's environment from it. The installed copy
of this file finds the tree through share/rover_sim/source_dir, which CMakeLists.txt writes.
"""
import os
import re
import shutil
import sys
from pathlib import Path
from tempfile import mkdtemp

import launch.logging
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, Shutdown)
from launch.event_handlers import OnProcessExit, OnProcessIO, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def source_dir(here=None):
    """rover_sim's source tree: the folder above this file's when it runs from there, else the one CMake recorded in
    the installed share/rover_sim/source_dir."""
    package = Path(here or __file__).resolve().parent.parent
    if (package / "gzenv.py").is_file():
        return package
    return Path((package / "source_dir").read_text().strip())


SIM_DIR = source_dir()
sys.path[:0] = [str(SIM_DIR), str(SIM_DIR / "tools")]
import fetch_data  # noqa: E402
import gen_rover  # noqa: E402
import gzenv  # noqa: E402
import rover_config  # noqa: E402
from urc import sdf  # noqa: E402

WORLDS = SIM_DIR / "worlds"
ROVER_URI = "<uri>model://rover</uri>"
SENSORS = re.compile(r'<plugin filename="gz-sim-sensors-system".*?</plugin>', re.S)
CONTROLLERS_LAUNCH = rover_config.CONTROL_DIR / "launch" / "controllers.launch.py"
STATION_WAIT = 120  # [s] for the world to come up before an attached station gives up (a big world loads slowly)
# RoverSimSystem's line once the controller manager has initialized it (ros2_control/rover_sim_system.cpp, on_init),
# in the server's log: once at the start, and again after every world reset.
HARDWARE_UP = b"wheels driven through RoverDrivetrain's motor model"


def headless(platform=None, environ=None):
    """Whether there is no display to open a window on (Linux without X11 or Wayland)."""
    platform = sys.platform if platform is None else platform
    environ = os.environ if environ is None else environ
    return platform.startswith("linux") and not (environ.get("DISPLAY") or environ.get("WAYLAND_DISPLAY"))


def resolve_world(arg):
    """The .sdf of a name in rover_sim/worlds or of a path; a world that is not here says why."""
    path = Path(arg)
    if not path.is_file():
        path = WORLDS / f"{arg}.sdf"
    if not path.is_file():
        raise FileNotFoundError(f"no world {arg!r}: " + (fetch_data.world_missing(arg) or
                                                         "give an .sdf path or a name in rover_sim/worlds"))
    return path.resolve()


def world_name(text):
    return re.search(r'<world\s+name="([^"]+)"', text).group(1)


def ros_world(text, rover_uri, sensors=True):
    """The world `text` with its one model://rover include pointed at rover_uri (its name and pose kept), and without
    Gazebo's Sensors system unless `sensors`."""
    count = text.count(ROVER_URI)
    if count != 1:
        raise ValueError(f"the world includes model://rover {count} times: sim.launch.py swaps exactly one")
    text = text.replace(ROVER_URI, f"<uri>{rover_uri}</uri>")
    return text if sensors else SENSORS.sub("", text)


def prepare(world, sensors, directory):
    """The rover with ros2_control (gen_rover: the post-processed model the equivalence test vouches for, not Gazebo's
    own conversion of /robot_description) and the world's copy, written into `directory`: (the copy, the URDF)."""
    urdf = gen_rover.build_urdf(rover_config.Variant(), ros2_control=True)
    model = directory / "rover"
    model.mkdir()
    (model / "model.sdf").write_text(gen_rover.convert(urdf))
    (model / "model.config").write_text(sdf.model_config("rover", "The rover with ros2_control.",
                                                         "rover_sim/launch/sim.launch.py"))
    copy = directory / world.name
    copy.write_text(ros_world(world.read_text(), model.as_uri(), sensors))
    return copy, urdf


def controllers(controller, ros_home):
    """rover_control's controllers.launch.py with the drive controller `controller`, its spawner in a ROS_HOME of its
    own, `ros_home` (gzenv.private_ros_home: in ~/.ros its lock file would be every spawner's on the machine)."""
    return GroupAction([
        *(SetEnvironmentVariable(name, value) for name, value in gzenv.private_ros_home(ros_home).items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(CONTROLLERS_LAUNCH)), launch_arguments={
            "controller": controller, "controllers": str(rover_config.CONTROLLERS_YAML)}.items()),
    ])


class RespawnAfterReset:
    """The handler of the server's output that spawns the controllers again, spawn(), each time RoverSimSystem comes
    up (HARDWARE_UP) after the first time. gz_ros2_control 1.2.19 does not implement Gazebo's reset (ISystemReset), so
    a world reset unloads and loads it again: its new controller manager loads the rover's hardware but no
    controllers, and the spawner of the start has exited."""

    def __init__(self, spawn):
        self.spawn = spawn
        self.seen = 0
        self.rest = {}  # stream -> the end of its output after the last newline

    def __call__(self, event):
        stream = "stdout" if event.from_stdout else "stderr"
        *lines, self.rest[stream] = (self.rest.get(stream, b"") + event.text).split(b"\n")
        actions = []
        for line in lines:
            if HARDWARE_UP in line:
                self.seen += 1
                if self.seen > 1:
                    actions += [LogInfo(msg="the world was reset: gz_ros2_control started a new controller manager, "
                                            "so the controllers are spawned again"), self.spawn()]
        return actions


def stop_if_a_spawner_fails(event, context):
    """A controller spawner that exits with an error (the controller manager did not come up, or could not load the
    rover's hardware) leaves a world nothing can drive: say so and stop. Not while the launch is stopping anyway."""
    action = event.action
    if context.is_shutdown or event.returncode == 0 or getattr(action, "node_executable", None) != "spawner":
        return None
    reason = (f"the controllers did not start (the spawner exited with code {event.returncode}), so nothing can "
              "drive the rover: the lines above say why. Start the simulation again; gz_ros2_control's 'Could not "
              "load library ... dlclose(0x0): invalid handle' is a rare race in ROS 2's library loader on macOS "
              "(docs/sim/manual.md, Known limitations)")
    launch.logging.get_logger("sim.launch").error(reason)
    return [Shutdown(reason=reason)]


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    world = resolve_world(arg("world"))
    directory = Path(mkdtemp(prefix=f"rover_sim_launch_{world.stem}_"))
    copy, urdf = prepare(world, arg("sensors") == "true", directory)
    env = gzenv.environment()
    server = ExecuteProcess(cmd=["gz", "sim", "-s", "-r", str(copy)], env=env, output="screen", name="gz_server")
    controller, ros_home = arg("controller"), directory / "ros_home"
    respawn = RespawnAfterReset(lambda: controllers(controller, ros_home))
    actions = [
        LogInfo(msg=f"world: {copy} (a copy of {world}, the rover with ros2_control)"),
        server,
        RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[Shutdown(reason="Gazebo's server exited")])),
        RegisterEventHandler(OnProcessIO(target_action=server, on_stdout=respawn, on_stderr=respawn)),
        RegisterEventHandler(OnProcessExit(on_exit=stop_if_a_spawner_fails)),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda _: shutil.rmtree(directory, ignore_errors=True))])),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": urdf, "use_sim_time": True}]),
        controllers(controller, ros_home),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": str(SIM_DIR / "bridge.yaml"), "use_sim_time": True}]),
    ]
    if arg("gui") == "true":
        gui = ExecuteProcess(cmd=["gz", "sim", "-g"], env=env, output="screen", name="gz_gui")
        actions += [gui, RegisterEventHandler(OnProcessExit(target_action=gui,
                                                            on_exit=[Shutdown(reason="Gazebo's GUI closed")]))]
    if arg("station") == "true":
        actions.append(ExecuteProcess(
            cmd=[sys.executable, str(SIM_DIR / "station" / "__main__.py"), "--world-name", world_name(copy.read_text()),
                 "--wait", str(STATION_WAIT)], env=env, output="screen", name="station"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="rover_test",
                              description="a name in rover_sim/worlds or an .sdf path"),
        DeclareLaunchArgument("gui", default_value="false" if headless() else "true",
                              description="Gazebo's GUI, a separate process"),
        DeclareLaunchArgument("station", default_value="false", description="the driver station, attached"),
        DeclareLaunchArgument("controller", default_value="drive_controller",
                              description="the drive controller in rover_control's controllers.yaml"),
        DeclareLaunchArgument("sensors", default_value="true",
                              description="Gazebo's Sensors system (false: nothing renders, no GPU needed)"),
        OpaqueFunction(function=launch_setup),
    ])
