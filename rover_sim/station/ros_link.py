"""The station's ROS 2 side: the drive command, geometry_msgs/TwistStamped on /cmd_vel (the topic contract,
docs/topics.md), which the drive controller takes on the simulated rover as on the real one (rover_control's
config/controllers.yaml). The header's stamp stays zero: diff_drive_controller stamps a zero-stamped command with its
own clock when it arrives (sim time in the simulation), so its cmd_vel_timeout counts from then whatever clock the
station has; a wall-clock stamp under sim time would never time out."""
import os
import threading

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.signals import SignalHandlerOptions

CMD_VEL_TOPIC = "/cmd_vel"
UNHEARD = 10.0  # [s] after the start with nothing subscribed to /cmd_vel, the station says so


class CmdVel:
    """twist(vx, wz) on /cmd_vel: the drive output (GzLink's drive) of a station in the ROS stack. It needs no executor:
    the station publishes from its own threads, and nothing here subscribes. After close() nothing uses the node: a
    lock keeps the unheard-warning's timer and the station's threads from it while it is destroyed."""

    def __init__(self):
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            # No rclpy signal handlers: the station stops on Ctrl-C, SIGTERM and SIGHUP itself and sends a zero twist
            # on its way out (server.Station.shutdown); rclpy's own handlers would shut its context down first, and
            # that publish would fail ("publisher's context is invalid").
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node(f"driver_station_{os.getpid()}")
        self._publisher = self.node.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)
        self._msg = TwistStamped()
        self._lock = threading.RLock()
        self._closed = False
        self._timers = []

    def twist(self, vx, wz):
        with self._lock:
            if self._closed or not rclpy.ok():
                return  # closed, or rclpy is down (another owner of the context shut it): nothing can go out
            self._msg.twist.linear.x, self._msg.twist.angular.z = float(vx), float(wz)
            self._publisher.publish(self._msg)

    def subscribers(self):
        """How many subscriptions to /cmd_vel the ROS graph has (the drive controller's, once it is up); 0 once
        closed."""
        with self._lock:
            return 0 if self._closed else self._publisher.get_subscription_count()

    def warn_if_unheard(self, after=UNHEARD, out=print):
        """After `after` seconds say so, through out(line), if nothing subscribes to /cmd_vel: no drive controller,
        as in a world started without sim.launch.py. Not once closed: close() cancels it (a station stopped before
        then may still be waiting for its world to stop). Returns the timer (a daemon thread)."""
        def check():
            with self._lock:
                if self._closed:
                    return
                unheard = self.subscribers() == 0
            if unheard:
                out(f"nothing subscribes to {CMD_VEL_TOPIC} after {after:.0f} s: the drive controller is not up (it "
                    "comes with sim.launch.py: pixi run sim, pixi run drive <world>), so the rover will not move")

        timer = threading.Timer(after, check)
        timer.daemon = True
        with self._lock:
            self._timers.append(timer)
        timer.start()
        return timer

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            for timer in self._timers:
                timer.cancel()
            self.node.destroy_node()
            if self._owns_context and rclpy.ok():
                rclpy.shutdown()
