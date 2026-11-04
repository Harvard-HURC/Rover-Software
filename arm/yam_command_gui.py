"""Optional Qt test console, imported only when the ROS node has gui:=true."""

import math
import queue
import threading
import time
from typing import TYPE_CHECKING, Callable

import rclpy
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QCloseEvent, QKeySequence
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QShortcut,
    QVBoxLayout,
    QWidget,
)
from rclpy.executors import ExternalShutdownException

from yam_command_help import COMMAND_USAGE
from yam_commands import JOINT_NAMES, motion_duration, parse_duration, positions

if TYPE_CHECKING:
    from yam_command_sender import YamCommandSender

FRESH_SECONDS = 0.8


class CommandWindow(QWidget):
    def __init__(self, events: queue.Queue, commands: queue.Queue, on_close: Callable[[], None]) -> None:
        super().__init__()
        self.events = events
        self.commands = commands
        self.on_close = on_close
        self.snapshot: dict = {}
        self.received_at = 0.0
        self.pending = False
        self.initialized = False
        self.closed = False
        self.help_dialog: QDialog | None = None
        self.mode = QLabel("WAITING FOR SENDER")
        self.connection = QLabel("Connecting to the joint-state sender...")
        self.bounds = QLabel("Start the joint-state sender with command reception enabled.")
        self.step = QComboBox()
        self.step.setEditable(True)
        self.step.addItems(["0.01", "0.025", "0.05", "0.1"])
        self.step.setCurrentText("0.05")
        self.duration = QComboBox()
        self.duration.setEditable(True)
        self.duration.addItems(["auto", "instant", "0.5", "1", "2", "5"])
        self.duration.setAccessibleName("Move and jog duration in seconds")
        self.duration.setToolTip(
            "Seconds for moves and jogs. Auto, instant, or 0 uses the fastest speed-limited ramp."
        )
        self.current = [QLabel("—") for _ in JOINT_NAMES]
        self.targets = [QLineEdit() for _ in JOINT_NAMES]
        self.range_labels = [QLabel("—") for _ in JOINT_NAMES]
        self.motion_buttons: list[QPushButton] = []
        self.jog_buttons: list[tuple[QPushButton, QPushButton]] = []
        self.build()
        self.escape = QShortcut(QKeySequence("Escape"), self, activated=self.hold)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(50)
        self.refresh()

    def build(self) -> None:
        self.setWindowTitle("YAM Arm · Test console")
        self.resize(950, 780)
        self.setMinimumSize(880, 720)
        self.setStyleSheet("""
            QWidget { background: #101822; color: #e5edf5; font-size: 14px; }
            QLabel#eyebrow, QLabel#mode { color: #62dec3; font-size: 12px; font-weight: 600; }
            QLabel#title { font-size: 30px; font-weight: 600; }
            QLabel[muted="true"] { color: #9aaec1; font-size: 12px; }
            QLineEdit, QComboBox { background: #1b2a3a; border: 1px solid #3b4e64;
                border-radius: 5px; padding: 8px; }
            QLineEdit:focus { border-color: #62dec3; }
            QComboBox QAbstractItemView { background: #1b2a3a; selection-background-color: #237f70; }
            QPushButton { background: #233549; border: 1px solid #3b4e64;
                border-radius: 5px; padding: 10px 16px; }
            QPushButton:hover { background: #354c64; }
            QPushButton:disabled { color: #728497; background: #1b2733; border-color: #283746; }
            QPushButton#send { background: #237f70; color: white; }
            QPushButton#send:hover { background: #299c87; }
            QPushButton#send:disabled { color: #728497; background: #29403e; }
            QPushButton#hold { background: #a43c48; color: white; font-weight: 600; font-size: 15px; }
            QPushButton#hold:hover { background: #c54b59; }
            QPlainTextEdit { background: #0b121a; color: #bdcedd; border: 1px solid #243444;
                border-radius: 6px; padding: 8px; font-size: 12px; }
        """)
        body = QVBoxLayout(self)
        body.setContentsMargins(26, 24, 26, 24)
        body.setSpacing(10)
        eyebrow = QLabel("YAM / ARM TEST CONSOLE")
        eyebrow.setObjectName("eyebrow")
        body.addWidget(eyebrow)
        title = QLabel("Joint commands")
        title.setObjectName("title")
        header = QHBoxLayout()
        header.addWidget(title)
        header.addStretch()
        self.help_button = QPushButton("Commands && usage")
        self.help_button.clicked.connect(self.show_commands)
        header.addWidget(self.help_button)
        body.addLayout(header)
        self.mode.setObjectName("mode")
        body.addWidget(self.mode)
        self.connection.setWordWrap(True)
        body.addWidget(self.connection)
        self.bounds.setProperty("muted", True)
        self.bounds.setWordWrap(True)
        body.addWidget(self.bounds)
        body.addSpacing(8)
        table = QGridLayout()
        table.setHorizontalSpacing(20)
        table.setVerticalSpacing(9)
        for column, text in enumerate(("JOINT", "MEASURED · rad", "TARGET · rad", "MODEL RANGE · rad", "JOG")):
            label = QLabel(text)
            label.setProperty("muted", True)
            table.addWidget(label, 0, column)
        table.setColumnStretch(3, 1)
        for index, name in enumerate(JOINT_NAMES):
            row = index + 1
            table.addWidget(QLabel(name), row, 0)
            table.addWidget(self.current[index], row, 1)
            self.targets[index].setFixedWidth(130)
            self.targets[index].setAccessibleName(f"{name} target in radians")
            table.addWidget(self.targets[index], row, 2)
            self.range_labels[index].setProperty("muted", True)
            table.addWidget(self.range_labels[index], row, 3)
            jog = QHBoxLayout()
            minus, plus = QPushButton("-"), QPushButton("+")
            for button, direction in ((minus, -1), (plus, 1)):
                button.setFixedWidth(44)
                button.setAccessibleName(f"Jog {name} {'positive' if direction > 0 else 'negative'}")
                button.clicked.connect(lambda checked=False, i=index, d=direction: self.jog(i, d))
                jog.addWidget(button)
            table.addLayout(jog, row, 4)
            self.jog_buttons.append((minus, plus))
            self.motion_buttons.extend((minus, plus))
        body.addLayout(table)
        body.addSpacing(8)
        timing = QHBoxLayout()
        timing.addWidget(QLabel("Duration (s)"))
        self.duration.setFixedWidth(120)
        timing.addWidget(self.duration)
        timing_note = QLabel("Moves & jogs · auto / instant = fastest within speed limit")
        timing_note.setProperty("muted", True)
        timing.addWidget(timing_note)
        timing.addStretch()
        body.addLayout(timing)
        actions = QHBoxLayout()
        actions.addWidget(QLabel("Jog step (rad)"))
        self.step.setFixedWidth(100)
        self.step.setAccessibleName("Jog step in radians")
        actions.addWidget(self.step)
        actions.addStretch()
        self.send_button = QPushButton("Send targets")
        self.send_button.setObjectName("send")
        self.send_button.clicked.connect(self.send_targets)
        actions.addWidget(self.send_button)
        self.copy_button = QPushButton("Copy measured")
        self.copy_button.clicked.connect(self.copy_current)
        actions.addWidget(self.copy_button)
        self.motion_buttons.append(self.send_button)
        body.addLayout(actions)
        self.hold_button = QPushButton("HOLD CURRENT POSITION   ·   Esc")
        self.hold_button.setObjectName("hold")
        self.hold_button.clicked.connect(self.hold)
        body.addWidget(self.hold_button)
        note = QLabel("Editing values does not move the arm. Hold uses position control; it is not an emergency stop.")
        note.setProperty("muted", True)
        note.setWordWrap(True)
        body.addWidget(note)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(150)
        self.log.setMinimumHeight(100)
        body.addWidget(self.log, 1)
        self.add_log("Ready to connect. Opening this window sends no motion commands.")

    def show_commands(self) -> None:
        if self.help_dialog is None:
            self.help_dialog = QDialog(self)
            self.help_dialog.setWindowTitle("YAM Arm · Commands & usage")
            self.help_dialog.resize(740, 680)
            layout = QVBoxLayout(self.help_dialog)
            reference = QPlainTextEdit(COMMAND_USAGE)
            reference.setReadOnly(True)
            reference.setAccessibleName("Commands and usage reference")
            reference.setStyleSheet("font-size: 14px;")
            layout.addWidget(reference)
            buttons = QDialogButtonBox(QDialogButtonBox.Close)
            buttons.rejected.connect(self.help_dialog.close)
            layout.addWidget(buttons)
        self.help_dialog.show()
        self.help_dialog.raise_()
        self.help_dialog.activateWindow()

    def add_log(self, text: str) -> None:
        self.log.appendPlainText(f"{time.strftime('%H:%M:%S')}  {text}")

    def fresh(self) -> bool:
        return bool(self.snapshot) and time.monotonic() - self.received_at < FRESH_SECONDS

    def can_move(self) -> bool:
        return (
            self.fresh()
            and self.snapshot.get("mode") in ("simulation", "hardware")
            and not self.pending
            and self.snapshot.get("status") != "moving"
        )

    def copy_current(self) -> None:
        if self.fresh():
            for target, current in zip(self.targets, self.snapshot["position"], strict=True):
                target.setText(f"{current:.6f}")

    def submit(self, operation: str, **fields: object) -> None:
        if operation != "hold" and not self.can_move():
            self.add_log("Wait for fresh feedback and for the current command to finish.")
            return
        if operation == "hold":
            try:
                self.commands.get_nowait()  # Hold replaces a move not yet dispatched.
            except queue.Empty:
                pass
        try:
            self.commands.put_nowait({"op": operation, "fields": fields, "expires": time.monotonic() + 0.5})
        except queue.Full:
            self.add_log("A command is already pending.")
            return
        self.pending = True
        self.update_controls()

    def send_targets(self) -> None:
        if not self.can_move():
            return
        try:
            target = positions([float(value.text()) for value in self.targets])
            current = positions(self.snapshot["position"])
            for index, (low, high) in enumerate(self.snapshot["limits"]):
                if not low <= target[index] <= high:
                    raise ValueError(f"{JOINT_NAMES[index]} must be between {low:.5f} and {high:.5f} radians")
            if any(abs(target - current) > self.snapshot["max_delta"] + 1e-9):
                raise ValueError(
                    f"Each target must be within {self.snapshot['max_delta']} rad of its measured position"
                )
            duration = parse_duration(self.duration.currentText())
            motion_duration(float(max(abs(target - current))), self.snapshot["speed"], duration)
            self.submit("move", position=target.tolist(), duration=duration)
        except (ValueError, KeyError) as exc:
            self.add_log(f"Not sent: {exc}")

    def jog(self, index: int, direction: int) -> None:
        if not self.can_move():
            return
        try:
            step = float(self.step.currentText())
            if not math.isfinite(step) or not 0 < step <= self.snapshot["max_delta"]:
                raise ValueError(f"Jog step must be between 0 and {self.snapshot['max_delta']} rad")
            duration = parse_duration(self.duration.currentText())
            motion_duration(step, self.snapshot["speed"], duration)
            self.submit("jog", joint=JOINT_NAMES[index], delta=direction * step, duration=duration)
        except ValueError as exc:
            self.add_log(f"Not sent: {exc}")

    def hold(self) -> None:
        self.submit("hold")

    def update_controls(self) -> None:
        for button in self.motion_buttons:
            button.setEnabled(self.can_move())
        self.copy_button.setEnabled(self.fresh())

    def refresh(self) -> None:
        if self.closed:
            return
        for _ in range(100):
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            if event["kind"] == "state":
                self.snapshot = event["data"]
                self.received_at = event["at"]
                for variable, value in zip(self.current, self.snapshot["position"], strict=True):
                    variable.setText(f"{value: .5f}")
                for variable, limits in zip(self.range_labels, self.snapshot.get("limits", []), strict=False):
                    variable.setText(f"{limits[0]: .3f}  to  {limits[1]: .3f}")
                if not self.initialized and self.fresh():
                    self.copy_current()
                    self.initialized = True
            elif event["kind"] == "disconnected":
                self.received_at = 0.0
            elif event["kind"] == "result":
                self.pending = False
            elif event["kind"] == "log":
                self.add_log(event["text"])
        if self.fresh():
            mode = self.snapshot.get("mode", "unknown")
            self.mode.setText(
                {"simulation": "SIMULATION · NO PHYSICAL ARM", "hardware": "HARDWARE · PHYSICAL ARM"}.get(
                    mode, "SENDER UPDATE REQUIRED"
                )
            )
            status = "Sending command..." if self.pending else self.snapshot["status"]
            if not self.pending and status == "moving" and "duration" in self.snapshot:
                status += f"  /  {self.snapshot.get('elapsed', 0):.2f} of {self.snapshot['duration']:.2f} s"
            self.connection.setText(f"Connected  /  {status}")
            self.bounds.setText(
                f"Max change: {self.snapshot.get('max_delta', '?')} rad per joint   ·   Max target speed: {self.snapshot.get('speed', '?')} rad/s   ·   Gripper unchanged"
            )
        else:
            self.mode.setText("DISCONNECTED · MOTION UNAVAILABLE")
            self.connection.setText("Waiting for fresh feedback. Start the joint-state sender with --enable-commands.")
            self.bounds.setText("Last displayed positions may be stale. Commands will not be queued for reconnection.")
        self.update_controls()

    def closeEvent(self, event: QCloseEvent) -> None:
        if not self.closed:
            self.closed = True
            self.timer.stop()
            if self.help_dialog is not None:
                self.help_dialog.close()
            self.on_close()
        event.accept()


def run_gui(node: "YamCommandSender") -> None:
    """Keep Qt on the main thread and network/ROS work on a dedicated executor thread."""
    app = QApplication.instance() or QApplication([])
    stop = threading.Event()
    errors: list[Exception] = []
    window = CommandWindow(node.ui_events, node.ui_commands, stop.set)

    def spin() -> None:
        try:
            while rclpy.ok() and not stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.05)
        except ExternalShutdownException:
            pass
        except Exception as exc:
            errors.append(exc)
        finally:
            stop.set()

    worker = threading.Thread(target=spin, name="yam-gui-ros")
    worker.start()

    def check_worker() -> None:
        if stop.is_set() or not rclpy.ok():
            window.close()

    timer = QTimer()
    timer.timeout.connect(check_worker)
    timer.start(100)
    window.show()
    try:
        app.exec_()
    finally:
        timer.stop()
        stop.set()
        worker.join()
        window.close()
    if errors:
        raise RuntimeError("ROS GUI worker stopped") from errors[0]
