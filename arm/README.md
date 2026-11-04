# YAM arm: shared ROS environment and controls

The feedback bridge, CAN sender, ROS command node, and optional Qt console run directly from this folder.
Use Ubuntu 24.04 on x86-64 Linux or Windows with Ubuntu in WSL2. GUI mode uses a desktop display/WSLg;
the terminal interface and headless tests do not need one. Native Windows and macOS arm setup are not
validated; the `yam-*` tasks are Linux-only. No colcon build or rover simulation data is needed.

## Clone and install

Install Git, [Pixi](https://pixi.prefix.dev/latest/installation/) (tested with 0.81.0),
and [uv](https://docs.astral.sh/uv/getting-started/installation/) inside Linux/WSL.
For native dependency builds, install `build-essential` and `cmake` with apt.
Clone on the Linux filesystem for faster installation:

```bash
mkdir -p ~/hurc_ws
git clone --recurse-submodules --branch feat/yam-arm-setup https://github.com/Harvard-HURC/Rover-Software.git ~/hurc_ws/src
cd ~/hurc_ws/src
pixi install --locked
pixi run --locked yam-setup
pixi run --locked yam-test
```

After this branch is merged, omit `--branch feat/yam-arm-setup` for new clones. In an existing clone:

```bash
git fetch origin
git switch feat/yam-arm-setup
git pull --ff-only
git submodule update --init --recursive
pixi install --locked
pixi run --locked yam-setup
```

The root `pixi.lock` supplies ROS 2 Jazzy, its message types, NumPy, Qt, RViz and Foxglove. The I2RT
submodule at `arm/vendor/i2rt` is pinned to `120c3c81400171174604e503943f8d1ebc891058`, the revision
used by the original local setup. Its own `uv.lock` supplies the separate hardware/MuJoCo environment
using Python 3.11.17. `yam-setup` initializes the submodule and installs with `uv sync --locked`;
it does not attach USB, configure CAN, or open the arm. Keep the two Python environments separate:
the hardware driver pins NumPy 2.2.6, while the ROS environment uses NumPy 2.5.3.
Both locks reproduce the software dependencies; each computer still needs its own USB/CAN attachment
and network settings. The existing rover application and package versions are unchanged.

Do not source another ROS installation or the old computer's `install/local_setup.bash`. These scripts
use standard ROS messages and need no overlay. Commit dependency manifests and locks together when
intentionally updating them; ordinary setup must not re-resolve dependencies. See the
[Pixi install options](https://pixi.prefix.dev/latest/reference/cli/pixi/install/) and
[uv sync options](https://docs.astral.sh/uv/reference/cli/#uv-sync).

## Try it without the arm

In **each** terminal, `cd ~/hurc_ws/src` and set the same discovery settings:

```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

Choose a domain unused by other local ROS work. Start these in separate terminals, in order:

```bash
pixi run --locked yam-bridge   # UDP feedback -> /yam/joint_states
pixi run --locked yam-sim      # simulated arm, with commands enabled
pixi run --locked yam-command  # interactive command node
```

Use `pixi run --locked yam-gui` instead of `yam-command` for the graphical console. Choose one command
node at a time. In the terminal console, type `state`, `help`, `jog joint1 0.05 --duration 1`, `hold`,
or `quit`. Angles are radians; `move q1 q2 q3 q4 q5 q6` takes six absolute targets. Use targets appropriate
to the measured pose. The GUI shows SIMULATION or HARDWARE and provides targets, jog, duration and Hold.

`yam-test` uses domain 187, localhost discovery, temporary command ports, an offscreen GUI, and the
MuJoCo simulation. It never opens CAN. It checks command validation, limits, interpolation, lease loss,
ROS dispatch, stale-message rejection, feedback, hold, and GUI behavior.

## Connect the physical arm

The driver configuration is **YAM v1 + LINEAR_4310 gripper**, on `can0` at **1,000,000 bit/s**, matching
the original setup. Confirm that hardware configuration before starting. Only one process may own the
arm's CAN connection. Stop `yam-sim` and any previous arm driver first.

On Windows, install usbipd-win and keep an Ubuntu terminal open. In PowerShell:

```powershell
usbipd list
# Replace 4-4 with this machine's CAN adapter BUSID.
# Run bind in an Administrator PowerShell, once for an unshared adapter:
usbipd bind --busid 4-4
# Attach in a normal PowerShell:
usbipd attach --wsl --busid 4-4
```

Use the adapter's current BUSID; it is not a portable device identity. Follow Microsoft's
[WSL USB guide](https://learn.microsoft.com/en-us/windows/wsl/connect-usb) for installation and permissions.
On native Linux, plug in the CAN adapter directly. In Linux/WSL:

```bash
sudo modprobe gs_usb
ip -brief link
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

If `gs_usb` is unavailable, update WSL and check that its kernel includes the adapter's driver. If no
`can0` appears, verify USB attachment and the adapter rather than starting the sender. For a different
interface, pass `--channel can1` to the sender task and configure that interface instead.

Start `yam-bridge`, then **`pixi run --locked yam-hardware`**, then `yam-command` or `yam-gui`, in
separate terminals using the same ROS settings as above. `yam-feedback` starts the hardware sender
without accepting commands. Hardware startup enables the driver and its gravity compensation even in
feedback mode; it is not a passive CAN sniffer.

Commands preserve the gripper position, enforce model limits, limit each move to 0.5 rad per joint,
and interpolate at up to 0.25 rad/s. Lower limits with `--max-delta` and `--speed` on the sender task.
A lost command client's 0.8 s lease triggers hold. Software hold is position control, not an emergency
stop, and there is no collision checking. Keep the arm clear and the hardware stop accessible.

To finish: `hold`, then quit the command node, stop the hardware sender with Ctrl+C, and stop the bridge.
Stopping the hardware sender disables motor torque; support the arm as required. Then:

```bash
sudo ip link set can0 down
```

On Windows, `usbipd detach --busid 4-4` releases the selected adapter.

## ROS interfaces and other computers

| Interface | Type | Meaning |
| --- | --- | --- |
| `/yam/joint_states` | `sensor_msgs/msg/JointState` | Measured positions for joint1 through joint6, approximately 10 Hz |
| `/yam/joint_commands` | `sensor_msgs/msg/JointState` | Six named position targets, each joint exactly once, radians |
| `/yam/hold` | `std_srvs/srv/Trigger` | Request and acknowledge software hold |
| `/yam/command_status` | `std_msgs/msg/String` | Acknowledgements, errors and state transitions |

Joint commands require a wall-clock header stamp at most 0.5 s old, empty velocity/effort, and reliable,
volatile QoS. The command node's `move_duration` parameter selects the duration for ROS topic commands;
terminal and GUI commands select their own duration. `0`/`auto`/`instant` means the fastest bounded ramp,
not an instantaneous jump. The gripper is not exposed as a commandable ROS joint here.

```bash
pixi run --locked ros2 topic echo /yam/joint_states
pixi run --locked ros2 service call /yam/hold std_srvs/srv/Trigger '{}'
pixi run --locked ros2 topic echo /yam/command_status
```

The hardware sender, feedback bridge and command node must run on the **same Linux host/WSL distribution**:
their UDP transport is loopback-only (feedback 8765, commands 8766). For a teammate on another computer,
keep `yam-daemon` running on the arm computer instead of the terminal/GUI node. On both hosts, use the
agreed ROS domain, unset `ROS_LOCALHOST_ONLY`, and set `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`. Synchronize
clocks for the 0.5 s stamp check. Verify discovery with `ros2 topic list` and feedback before publishing.
DDS multicast/firewall rules and WSL networking must allow communication; a Git pull alone cannot set
that up. Any publisher in that reachable ROS domain can issue arm commands; use the team's trusted network.
The current GUI talks to a local sender and cannot directly control another host's UDP endpoint.

## Visualization and ShellWeaver

With the bridge running, `pixi run --locked yam-description` publishes `/yam/robot_description` and
TF frames prefixed `yam/`. `pixi run --locked yam-rviz` opens RViz with fixed frame `yam/base`; add a
RobotModel display using `/yam/robot_description` with transient-local durability. The pinned upstream
arm model supplies the meshes; it does not visualize the separately composed LINEAR_4310 gripper.
Optional `pixi run --locked yam-foxglove` serves the graph at `ws://127.0.0.1:8765` (TCP WebSocket,
distinct from the feedback UDP port). It binds loopback by default.

For ShellWeaver, create WSL terminal profiles with the Linux clone as the working directory and the
same domain/discovery exports followed by the corresponding `pixi run --locked yam-*` command. Long-running
tasks are services. Bridge Ready phrase: `Listening for i2rt joint feedback on UDP port 8765`; sender:
`Connected. Sending joint feedback to`; command node: `Joint targets: /yam/joint_commands`.
Set the ROS viewer working directory to that clone, Python command to `pixi run --locked python`, and
domain to the chosen domain. If Pixi is not on PATH, use `"$HOME/.pixi/bin/pixi"`. No old absolute paths,
saved passwords, user profiles, compiled environments, or personal app data are needed.
