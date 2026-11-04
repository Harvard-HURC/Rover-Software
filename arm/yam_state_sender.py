"""Publish feedback and optionally accept commands on the same CAN connection."""

import json
import socket
import time

import tyro

from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import ArmType, GripperType
from yam_commands import COMMAND_PORT, CommandServer, JointController, positions

HOST = "127.0.0.1"
PORT = 8765


def main(
    enable_commands: bool = False,
    sim: bool = False,
    channel: str = "can0",
    command_port: int = COMMAND_PORT,
    feedback_port: int = PORT,
    max_delta: float = 0.5,
    speed: float = 0.25,
) -> None:
    """Run feedback; --enable-commands adds commands, --sim avoids hardware.

    max_delta: Maximum change per command in radians (up to 0.5).
    speed: Maximum interpolated target speed in radians/second (up to 0.25).
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot = None
    server = None
    try:
        # Validate settings and claim the command port before touching hardware.
        controller = JointController(None, max_delta=max_delta, speed=speed, sim=sim)
        if enable_commands:
            server = CommandServer(command_port)
        print("Connecting to simulated YAM..." if sim else "Connecting to YAM...", flush=True)
        robot = get_yam_robot(
            channel=channel,
            arm_type=ArmType.YAM,
            gripper_type=GripperType.LINEAR_4310,
            sim=sim,
        )
        controller.robot = robot
        print(f"Connected. Sending joint feedback to {HOST}:{feedback_port}", flush=True)
        if server:
            print(f"Commands enabled on localhost:{command_port}; arm remains idle until commanded.", flush=True)
        next_feedback = 0.0
        while True:
            now = time.monotonic()
            if server:
                # Expire a lost client's lease before handling newly arrived commands.
                controller.tick(now)
                server.poll(controller)
            if now >= next_feedback:
                current = positions(robot.get_joint_pos(), 7)
                message = json.dumps({"position": current[:6].tolist()}, allow_nan=False).encode()
                sock.sendto(message, (HOST, feedback_port))
                next_feedback = now + 0.1
            time.sleep(0.01 if server else 0.1)
    except KeyboardInterrupt:
        print("\nStopping sender...", flush=True)
    finally:
        if server:
            server.close()
        sock.close()
        if robot is not None:
            robot.close()


if __name__ == "__main__":
    tyro.cli(main)
