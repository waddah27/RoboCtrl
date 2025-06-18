import sys
import os
import time
import threading
from queue import Queue
import math
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                               QLabel, QPushButton, QDoubleSpinBox, QGroupBox, QStatusBar)
from PySide6.QtCore import Qt, QTimer, Signal, QObject, Slot
from PySide6.QtGui import QPainter, QPen, QColor


class CNCSignals(QObject):
    position_changed = Signal(float)
    status_changed = Signal(dict)


class CNCAxisController:
    def __init__(self, num_motors=1, steps_per_mm=100, max_speed=500, acceleration=100):
        self.num_motors = num_motors
        self.steps_per_mm = steps_per_mm
        self.max_speed = max_speed  # mm/min
        self.acceleration = acceleration  # mm/s²
        self.current_position = 0.0  # mm
        self.target_position = 0.0  # mm
        self.is_moving = False
        self._emergency_stop = False  # <-- renamed here
        self.motor_positions = [0.0, 0.0]
        self.command_queue = Queue()
        self.signals = CNCSignals()
        self.control_thread = threading.Thread(target=self._motion_control_loop, daemon=True)
        self.control_thread.start()

    def move_to_position(self, position_mm):
        if not self._emergency_stop:
            self.command_queue.put(('MOVE_ABS', position_mm))

    def move_relative(self, distance_mm):
        if not self._emergency_stop:
            self.command_queue.put(('MOVE_REL', distance_mm))

    def stop(self):
        self.command_queue.put(('STOP', None))

    @Slot()
    def emergency_stop(self):  # now safely a method
        self._emergency_stop = True
        self.stop()

    @Slot()
    def reset_emergency(self):
        self._emergency_stop = False

    def _motion_control_loop(self):
        while True:
            if not self.command_queue.empty():
                cmd, value = self.command_queue.get()
                if cmd == 'MOVE_ABS':
                    self.target_position = value
                    self._execute_move()
                elif cmd == 'MOVE_REL':
                    self.target_position = self.current_position + value
                    self._execute_move()
                elif cmd == 'STOP':
                    self.is_moving = False
                    self.signals.status_changed.emit(self.get_status())
            time.sleep(0.001)

    def _execute_move(self):
        if self._emergency_stop:
            return
        distance = self.target_position - self.current_position
        if abs(distance) < 0.001:
            return
        self.is_moving = True
        self.signals.status_changed.emit(self.get_status())
        direction = 1 if distance > 0 else -1
        distance = abs(distance)
        max_speed_mms = self.max_speed / 60.0
        t_accel = max_speed_mms / self.acceleration
        d_accel = 0.5 * self.acceleration * t_accel**2
        if 2 * d_accel > distance:
            t_accel = math.sqrt(distance / self.acceleration)
            d_accel = 0.5 * self.acceleration * t_accel**2
            max_speed_mms = self.acceleration * t_accel
            t_decel = t_accel
            t_const = 0
        else:
            t_decel = t_accel
            t_const = (distance - 2 * d_accel) / max_speed_mms
        total_time = t_accel + t_const + t_decel
        start_time = time.time()
        elapsed = 0
        while elapsed < total_time and not self._emergency_stop:
            if elapsed < t_accel:
                speed = self.acceleration * elapsed
                distance_covered = 0.5 * self.acceleration * elapsed**2
            elif elapsed < t_accel + t_const:
                speed = max_speed_mms
                distance_covered = d_accel + max_speed_mms * (elapsed - t_accel)
            else:
                speed = max_speed_mms - self.acceleration * (elapsed - t_accel - t_const)
                distance_covered = (d_accel +
                                    max_speed_mms * t_const +
                                    max_speed_mms * (elapsed - t_accel - t_const) -
                                    0.5 * self.acceleration * (elapsed - t_accel - t_const)**2)
            self.current_position = self.target_position - direction * (distance - distance_covered)
            if self.num_motors == 2:
                self.motor_positions[0] = self.current_position
                self.motor_positions[1] = self.current_position
            self.signals.position_changed.emit(self.current_position)
            self._generate_step_pulses(direction, speed)
            time.sleep(0.001)
            elapsed = time.time() - start_time
        self.is_moving = False
        self.current_position = self.target_position
        self.signals.status_changed.emit(self.get_status())

    def _generate_step_pulses(self, direction, speed):
        steps_per_second = speed * self.steps_per_mm
        if steps_per_second > 0:
            step_delay = 1.0 / steps_per_second
            time.sleep(step_delay)

    def get_position(self):
        return self.current_position

    def get_status(self):
        return {
            'position': self.current_position,
            'target': self.target_position,
            'moving': self.is_moving,
            'emergency_stop': self._emergency_stop,
            'motors': self.num_motors
        }


class PositionIndicator(QWidget):
    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.setMinimumSize(400, 100)
        self.setMaximumHeight(120)
        self.controller.signals.position_changed.connect(self.update_position)
        self.position = 0.0
        self.target = 0.0
        self.is_moving = False

    def update_position(self, pos):
        self.position = pos
        self.update()

    def update_target(self, target):
        self.target = target
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(240, 240, 240))
        width = self.width()
        height = self.height()
        pen = QPen(Qt.black, 1)
        painter.setPen(pen)
        axis_y = height // 2
        painter.drawLine(20, axis_y, width - 20, axis_y)
        for x in range(20, width - 19, 50):
            painter.drawLine(x, axis_y - 5, x, axis_y + 5)
            painter.drawText(x - 10, axis_y + 20, f"{(x - 20) * 100 / (width - 40):.0f}")
        pos_x = 20 + (width - 40) * (self.position / 100)
        if self.is_moving:
            painter.setBrush(QColor(0, 150, 255))
        else:
            painter.setBrush(QColor(0, 200, 0))
        painter.drawEllipse(int(pos_x) - 8, axis_y - 8, 16, 16)
        if abs(self.target - self.position) > 0.1:
            target_x = 20 + (width - 40) * (self.target / 100)
            pen.setColor(Qt.red)
            pen.setWidth(2)
            painter.setPen(pen)
            painter.setBrush(Qt.transparent)
            painter.drawEllipse(int(target_x) - 10, axis_y - 10, 20, 20)


class CNCMainWindow(QMainWindow):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.setWindowTitle("CNC X-Axis Controller")
        self.setGeometry(100, 100, 800, 600)
        self.init_ui()
        self.connect_signals()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        self.position_indicator = PositionIndicator(self.controller)
        main_layout.addWidget(self.position_indicator)

        control_group = QGroupBox("Position Control")
        control_layout = QHBoxLayout()
        self.pos_spinbox = QDoubleSpinBox()
        self.pos_spinbox.setRange(-1000, 1000)
        self.pos_spinbox.setDecimals(3)
        self.pos_spinbox.setSuffix(" mm")
        self.pos_spinbox.setValue(0.0)
        self.move_abs_btn = QPushButton("Move Absolute")
        self.move_rel_btn = QPushButton("Move Relative")
        self.stop_btn = QPushButton("Stop")
        control_layout.addWidget(QLabel("Position:"))
        control_layout.addWidget(self.pos_spinbox)
        control_layout.addWidget(self.move_abs_btn)
        control_layout.addWidget(self.move_rel_btn)
        control_layout.addWidget(self.stop_btn)
        control_group.setLayout(control_layout)
        main_layout.addWidget(control_group)

        rel_group = QGroupBox("Relative Movement")
        rel_layout = QHBoxLayout()
        self.rel_spinbox = QDoubleSpinBox()
        self.rel_spinbox.setRange(-100, 100)
        self.rel_spinbox.setDecimals(3)
        self.rel_spinbox.setSuffix(" mm")
        self.rel_spinbox.setValue(10.0)
        self.move_left_btn = QPushButton("← Move Left")
        self.move_right_btn = QPushButton("Move Right →")
        rel_layout.addWidget(self.move_left_btn)
        rel_layout.addWidget(self.rel_spinbox)
        rel_layout.addWidget(self.move_right_btn)
        rel_group.setLayout(rel_layout)
        main_layout.addWidget(rel_group)

        emergency_group = QGroupBox("Emergency Controls")
        emergency_layout = QHBoxLayout()
        self.emergency_btn = QPushButton("EMERGENCY STOP")
        self.emergency_btn.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.reset_btn = QPushButton("Reset Emergency")
        emergency_layout.addWidget(self.emergency_btn)
        emergency_layout.addWidget(self.reset_btn)
        emergency_group.setLayout(emergency_layout)
        main_layout.addWidget(emergency_group)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.update_status()
        main_layout.addStretch()

    def connect_signals(self):
        self.move_abs_btn.clicked.connect(self.move_absolute)
        self.move_rel_btn.clicked.connect(self.move_relative)
        self.stop_btn.clicked.connect(self.controller.stop)
        self.emergency_btn.clicked.connect(self.controller.emergency_stop)
        self.reset_btn.clicked.connect(self.controller.reset_emergency)

        self.move_left_btn.clicked.connect(lambda: self.move_relative(-abs(self.rel_spinbox.value())))
        self.move_right_btn.clicked.connect(lambda: self.move_relative(abs(self.rel_spinbox.value())))

        self.controller.signals.position_changed.connect(self.update_position_display)
        self.controller.signals.status_changed.connect(self.update_status)

    def move_absolute(self):
        target = self.pos_spinbox.value()
        self.controller.move_to_position(target)

    def move_relative(self, distance=None):
        if distance is None:
            distance = self.rel_spinbox.value()
        self.controller.move_relative(distance)

    def update_position_display(self, position):
        self.pos_spinbox.setValue(position)
        self.position_indicator.update_position(position)

    def update_status(self, status=None):
        if status is None:
            status = self.controller.get_status()
        moving_text = "MOVING" if status['moving'] else "IDLE"
        emergency_text = " (EMERGENCY STOP ACTIVE)" if status['emergency_stop'] else ""
        self.status_bar.showMessage(
            f"Position: {status['position']:.3f} mm | "
            f"Target: {status['target']:.3f} mm | "
            f"Status: {moving_text}{emergency_text} | "
            f"Motors: {status['motors']}"
        )
        self.position_indicator.is_moving = status['moving']
        self.position_indicator.update_target(status['target'])
        self.position_indicator.update()


if __name__ == "__main__":
    if sys.platform == "linux":
        if "QT_QPA_PLATFORM" not in os.environ:
            os.environ["QT_QPA_PLATFORM"] = "xcb"
    app = QApplication(sys.argv)
    cnc_controller = CNCAxisController(num_motors=2, steps_per_mm=200, max_speed=1000, acceleration=200)
    window = CNCMainWindow(cnc_controller)
    window.show()
    sys.exit(app.exec())