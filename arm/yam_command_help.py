"""Shared command reference for the terminal and optional test console."""

COMMAND_SUMMARY = (
    "Commands: state | move q1 q2 q3 q4 q5 q6 | jog jointN delta | hold | commands | help | quit. "
    "Moves/jogs accept --duration SECONDS|auto|instant. Angles are radians. Type commands for usage and examples."
)

COMMAND_USAGE = """YAM COMMANDS & USAGE

TERMINAL COMMANDS
Type a command in the command sender terminal and press Enter.
All joint angles and jog increments are in radians, not degrees.

commands / help
    Show this reference. Does not send an arm command.

state
    Show status and the six measured joint positions in radians.

move q1 q2 q3 q4 q5 q6 [--duration SECONDS|auto|instant]
    Send six absolute targets, ordered joint1 through joint6.
    Replace every q with a finite number. Example syntax:
        move 0.1 1.0 1.2 -0.2 0.05 0.0
    Use targets appropriate to your measured pose and model limits;
    the example is not a recommended pose.

jog jointN delta [--duration SECONDS|auto|instant]
    Move one joint by a signed increment from its measured position.
    Joint names: joint1, joint2, joint3, joint4, joint5, joint6.
    Examples: jog joint1 0.05   or   jog joint1 -0.05

DURATION
    Omit --duration or use auto for the default speed-limited ramp.
    Example: jog joint1 0.05 --duration 1
    Example: move 0.1 1.0 1.2 -0.2 0.05 0.0 --duration 2
    Use instant or 0 for the fastest ramp within the speed limit:
        jog joint1 0.05 --duration instant
    Instant does not jump the target or bypass the speed limit.
    Positive durations must allow the configured target speed and
    a minimum 0.2-second ramp. Too-short requests are rejected with
    the minimum duration. Larger moves require more time.
    The acknowledgement reports the selected target duration.
    Timing describes the target ramp, not guaranteed physical arrival.
    Control-loop stalls can extend the ramp to avoid target jumps.

hold
    Cancel the current interpolation and hold the measured position.
    This is software position control, not an emergency stop.

quit
    Exit the command node; request hold if this node sent motion.
    Ctrl+C in the terminal also exits.

GUI CONTROLS
Copy measured: copy current feedback into the six target fields.
Send targets: send all six target fields as an absolute move.
Jog step: choose or enter a positive increment in radians.
Duration (s): enter seconds or choose auto / instant for moves and jogs.
The - / + buttons immediately send a jog for that row's joint.
Hold current position: request hold; Escape in the main window
also requests hold. Escape in this help window only closes help.
Commands & usage: open this reference without sending commands.

Editing target fields or the jog step does not move the arm.
Motion controls need fresh feedback and no pending/running move.
Closing the main window exits the node and requests hold if this
node sent motion. Closing this reference leaves the node running.

LIMITS & FEEDBACK
The gripper retains its position. Targets must satisfy model limits.
Default maximum change: 0.5 rad per joint from the measured pose.
Default maximum target speed: 0.25 rad/s. The sender can reduce
these limits; the main window shows the active limits.
A new move is rejected while moving; use hold to cancel first.
An acknowledgement means the target was accepted, not reached.
Check measured feedback. There is no collision checking or planning.

CONNECTION & ROS
Run yam_state_sender.py with --enable-commands in its hardware
Python environment. Add --sim for a hardware-free trial.
Run either the GUI or terminal command node in the ROS environment.
The GUI does not start the sender or switch its hardware/sim mode.

/yam/joint_commands (sensor_msgs/msg/JointState): six named joints,
positions in radians, empty velocity/effort, wall-clock header stamp
at most 0.5 seconds old. Names joint1 through joint6, each once.
ROS topic moves use the node's move_duration parameter (seconds,
default 0.0 for fastest). Example in a sourced ROS terminal:
    ros2 param set /yam_command_sender move_duration 1.0
Terminal commands and GUI controls use their own duration selection.
/yam/hold (std_srvs/srv/Trigger): request hold. Example:
    ros2 service call /yam/hold std_srvs/srv/Trigger '{}'
/yam/command_status (std_msgs/msg/String): status and rejections.
    ros2 topic echo /yam/command_status
"""
