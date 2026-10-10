"""Driving the simulated rover without ROS. The drivetrain's motors follow wheel speed setpoints
(plugins/rover_drivetrain.cpp; design spec 2026-10-09, section 6.1): on the rover with ros2_control, RoverSimSystem
writes them from the drive controller; without it they come on the drivetrain's gz-transport topic
rover_config.WHEEL_CMD_TOPIC, a gz.msgs.Double_V with one value per wheel in WHEELS order [rad/s].

The simulation's tests, tools and scripts speak body twists. This module turns a twist into the setpoints the
drivetrain itself made of it before the ros2_control boundary: each wheel at (vx - side x wz x track / 2) / radius
(side +1 left, -1 right; track 2 x rover.yaml rocker.pivot_y), clamped to the top speed and ramped at sim.yaml's
wheel_cmd accel, step by step with gz::math::SpeedLimiter's arithmetic as libgz-math7 computes it on osx-arm64 (one
rounding of previous + acceleration x dt, a fused multiply-add), so that a run that keeps commanding (Driver, every
step: the calibration runs) drives as it did, to the bit. Two kinds of run differ from before PR 3: one that stops
commanding stops harder, because the drivetrain's watchdog now zeroes a stale setpoint at once where it ramped the
twist down (docs/sim/design-notes.md, Watchdog), and the cost runs' simulate.twist_publisher ramps once per message
over the sim time of the 10 Hz world statistics, where the drivetrain ramped in every step. The ROS stack has its own
kinematics and ramp in the drive controller (rover_control/config/controllers.yaml), with the same numbers
(tests/test_wheel_cmd.py).

Driver feeds a run stepped in this process (gz.sim8.TestFixture); TwistDrive sends each twist as it comes, for a
caller whose twists are ramped already (the driver station slews its own).
"""
import math
from fractions import Fraction

from rover_config import CMD_VEL_TOPIC, ROVER, SIM, WHEEL_CMD_TOPIC

WHEELS = ("fl", "rl", "fr", "rr")  # the message's order: the drivetrain's <wheel> elements (urdf/rover_sim.urdf.xacro)
SIDES = (1.0, 1.0, -1.0, -1.0)  # +1 left, -1 right, in WHEELS order
STEP = 0.001  # [s] the worlds' physics step


def setpoints(vx, wz, rover=ROVER):
    """Each wheel's speed [rad/s] for the twist (vx [m/s], wz [rad/s]), in WHEELS order: the drivetrain's formula
    before PR 3, its operations in the same order, so the same doubles."""
    track, radius = 2 * rover.rocker.pivot_y, rover.wheel.radius
    return tuple((vx - side * wz * track / 2) / radius for side in SIDES)


def fused(previous, acceleration, dt):
    """previous + acceleration x dt rounded once to the nearest double, as libgz-math7's
    SpeedLimiter::LimitAcceleration computes it on osx-arm64 (compiled to a fused multiply-add, fmadd). Python 3.12
    has no math.fma: the exact rational sum, rounded once. With acceleration 0 the sum is previous, exactly."""
    if acceleration == 0.0:
        return previous + acceleration * dt
    return float(Fraction(previous) + Fraction(acceleration) * Fraction(dt))


class Ramp:
    """Each wheel's setpoint clamped to +-top_speed, then moved towards its target by at most accel [rad/s^2] (accel 0:
    no limit), as gz::math::SpeedLimiter's LimitVelocity and LimitAcceleration did in the drivetrain: the
    acceleration (target - previous) / dt is clamped, and the new setpoint is previous + acceleration x dt, rounded
    once (fused)."""

    def __init__(self, accel=SIM.wheel_cmd.accel, rover=ROVER):
        self.accel, self.top, self.rover = accel, rover.wheel.top_speed, rover
        self.values = (0.0,) * len(WHEELS)

    def step(self, vx, wz, dt=STEP):
        """The setpoints after a step of dt [s] towards the twist."""
        low, high = (-self.accel, self.accel) if self.accel > 0 else (-math.inf, math.inf)
        values = []
        for target, previous in zip(setpoints(vx, wz, self.rover), self.values):
            target = max(min(target, self.top), -self.top)
            values.append(fused(previous, max(min((target - previous) / dt, high), low), dt))
        self.values = tuple(values)
        return self.values

    def reset(self):
        self.values = (0.0,) * len(WHEELS)


def message(values):
    """The setpoints as the drivetrain's topic carries them."""
    from gz.msgs10.double_v_pb2 import Double_V  # here: the formula and the ramp need no Gazebo
    msg = Double_V()
    msg.data.extend(values)
    return msg


class Driver:
    """A twist schedule's setpoints for a run stepped in this process: command(vx, wz) when the twist changes
    (simulate sends one every 20 ms of sim time), step() once per physics step from the pre-update callback. step
    ramps (Ramp) and publishes the setpoints when they change, and with force (the 20 ms beat, so that the
    drivetrain's watchdog sees fresh ones), never before the first command. The drivetrain uses a setpoint published
    in a step from the next step on, as it used a twist. command also sends the twist as a gz.msgs.Twist on
    rover_config.CMD_VEL_TOPIC, which a rover with Gazebo's DiffDrive takes (sim.yaml drivetrain mode diffdrive) and
    the physical drivetrain ignores."""

    def __init__(self, node, accel=SIM.wheel_cmd.accel, rover=ROVER):
        from gz.msgs10.double_v_pb2 import Double_V
        from gz.msgs10.twist_pb2 import Twist
        self._wheels = node.advertise(WHEEL_CMD_TOPIC, Double_V)
        self._twists = node.advertise(CMD_VEL_TOPIC, Twist)
        self._twist = Twist()
        self.ramp = Ramp(accel, rover)
        self.twist = (0.0, 0.0)
        self._sent = None  # the setpoints last published; None: no command yet

    def command(self, vx, wz):
        self.twist = (vx, wz)
        self._twist.linear.x, self._twist.angular.z = vx, wz
        self._twists.publish(self._twist)
        if self._sent is None:
            self._sent = ()

    def step(self, force=False):
        if self._sent is None:
            return
        values = self.ramp.step(*self.twist)
        if force or values != self._sent:
            self._wheels.publish(message(values))
            self._sent = values

    def reset(self):
        """The world was reset: the ramp starts again from zero, and nothing goes out before the next command."""
        self.ramp.reset()
        self.twist = (0.0, 0.0)
        self._sent = None


class TwistDrive:
    """twist(vx, wz) as wheel setpoints without a ramp (setpoints), and as the twist for a DiffDrive rover: for a caller
    whose twists are ramped already, such as the driver station (station/drive.py slews its own), on a rover without
    ROS. The ROS stack takes the station's twist on /cmd_vel instead (station/ros_link.py)."""

    def __init__(self, node, rover=ROVER):
        from gz.msgs10.double_v_pb2 import Double_V
        from gz.msgs10.twist_pb2 import Twist
        self._wheels = node.advertise(WHEEL_CMD_TOPIC, Double_V)
        self._twists = node.advertise(CMD_VEL_TOPIC, Twist)
        self._twist = Twist()
        self.rover = rover

    def twist(self, vx, wz):
        self._twist.linear.x, self._twist.angular.z = vx, wz
        self._twists.publish(self._twist)
        self._wheels.publish(message(setpoints(vx, wz, self.rover)))
