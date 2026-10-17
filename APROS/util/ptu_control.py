import sys
import json
from typing import Optional

from PyQt6.QtCore import QTimer, QDateTime, Qt, QObject, pyqtSignal, QByteArray
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QGridLayout,
                             QHBoxLayout, QVBoxLayout, QGroupBox, QComboBox,
                             QLineEdit, QPushButton, QSlider, QLabel, QTextEdit,
                             QLCDNumber, QCheckBox, QDoubleSpinBox, QStatusBar)
from PyQt6.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt6.QtGui import QFont, QPixmap


class PelcoD:
    """Pelco-D 프로토콜 생성 및 파싱 클래스"""
    # --- Pelco-D Command Constants ---
    CMD_PAN_RIGHT = 0x02
    CMD_PAN_LEFT = 0x04
    CMD_TILT_UP = 0x08
    CMD_TILT_DOWN = 0x10
    CMD_STOP = 0x00

    SET_PAN_POSITION = 0x4B
    SET_TILT_POSITION = 0x4D
    SET_PRESET_SPEED = 0x55 # 프리셋 이동 속도 설정 (절대 위치 이동 속도 제어에 활용)

    QUERY_PAN = 0x51
    QUERY_TILT = 0x53
    RESPONSE_PAN = 0x59
    RESPONSE_TILT = 0x5B

    @staticmethod
    def calc_checksum(packet: list) -> int:
        return sum(packet[1:6]) % 256

    @staticmethod
    def make_command(addr: int, cmd1: int, cmd2: int, data1: int, data2: int) -> bytes:
        packet = [0xFF, addr, cmd1, cmd2, data1, data2]
        csum = PelcoD.calc_checksum(packet)
        packet.append(csum)
        return bytes(packet)

    @staticmethod
    def make_absolute_move(addr: int, move_type: str, angle: float, limits: tuple) -> bytes:
        min_angle, max_angle = limits
        angle = max(min_angle, min(max_angle, angle))

        if move_type == "TILT":
            if angle < 0:
                angle = 360.0 + angle
            cmd2 = PelcoD.SET_TILT_POSITION
        else:
            cmd2 = PelcoD.SET_PAN_POSITION

        val = int(angle * 100)
        data1 = (val >> 8) & 0xFF
        data2 = val & 0xFF
        return PelcoD.make_command(addr, 0x00, cmd2, data1, data2)

    @staticmethod
    def parse_response(rx_bytes: bytes):
        if len(rx_bytes) < 7 or rx_bytes[0] != 0xFF:
            return None, None

        packet = list(rx_bytes)
        if PelcoD.calc_checksum(packet[:6]) != packet[6]:
            return None, None

        cmd2 = packet[3]
        val = (packet[4] << 8) + packet[5]
        angle = val / 100.0

        if cmd2 == PelcoD.RESPONSE_PAN:
            return "PAN", angle
        elif cmd2 == PelcoD.RESPONSE_TILT:
            if angle > 180.0:
                angle = angle - 360.0
            return "TILT", angle
        return None, None

class PTUManager(QObject):
    """PTU 제어 로직 및 상태 관리를 담당하는 API 클래스"""
    # --- Signals ---
    connection_changed = pyqtSignal(bool)
    position_updated = pyqtSignal(str, float)
    log_message = pyqtSignal(str, str)
    absolute_move_finished = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._serial = QSerialPort()
        self._serial.readyRead.connect(self._read_serial_data)

        self._device_id = 0x01
        self._is_connected = False
        self._pan_angle = 0.0
        self._tilt_angle = 0.0
        self._rx_buffer = QByteArray()

        self._query_timer = QTimer(self)
        self._query_timer.timeout.connect(self.query_position)
        self._query_toggle = True
        self._auto_query_enabled = False
        self._auto_query_interval = 150

        # Absolute move monitoring state
        self._is_abs_moving = False
        self._abs_target_pan = 0.0
        self._abs_target_tilt = 0.0
        self._abs_last_pan = None
        self._abs_last_tilt = None
        self._abs_pan_no_change_count = 0
        self._abs_tilt_no_change_count = 0
        self._abs_start_time = 0

        self._abs_timer = QTimer(self)
        self._abs_timer.timeout.connect(self._abs_monitor_tick)

    @staticmethod
    def get_available_ports() -> list[str]:
        return [p.portName() for p in QSerialPortInfo.availablePorts()]

    def connect(self, port: str, baud_rate: int, device_id: int) -> bool:
        self._device_id = device_id
        if self._serial.isOpen():
            self._serial.close()
        self._serial.setBaudRate(baud_rate)
        self._serial.setDataBits(QSerialPort.DataBits.Data8)
        self._serial.setParity(QSerialPort.Parity.NoParity)
        self._serial.setStopBits(QSerialPort.StopBits.OneStop)
        self._serial.setPortName(port)

        if self._serial.open(QSerialPort.OpenModeFlag.ReadWrite):
            self._is_connected = True
            self.connection_changed.emit(True)
            self.log_message.emit("SYSTEM", f"Connected to {port}")
            return True
        else:
            self._is_connected = False
            self.connection_changed.emit(False)
            self.log_message.emit("ERROR", f"Failed to open port {port}")
            return False

    def disconnect(self):
        if self._is_connected:
            self.stop_auto_query()
            if self._is_abs_moving:
                self._stop_abs_monitoring()
            self._serial.close()
            self._is_connected = False
            self.connection_changed.emit(False)
            self.log_message.emit("SYSTEM", "Disconnected")

    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def pan_angle(self) -> float:
        return self._pan_angle

    @property
    def tilt_angle(self) -> float:
        return self._tilt_angle

    def _send_packet(self, packet: bytes, desc=""):
        if self.is_connected():
            self._serial.write(packet)
            hex_str = " ".join(f"{b:02X}" for b in packet)
            self.log_message.emit("Tx", f"{hex_str} ({desc})")

    def move(self, pan_speed: int, tilt_speed: int):
        if self._query_timer.isActive():
            self._query_timer.stop()

        cmd_dir = 0x00
        if pan_speed > 0: cmd_dir |= PelcoD.CMD_PAN_RIGHT
        if pan_speed < 0: cmd_dir |= PelcoD.CMD_PAN_LEFT
        if tilt_speed > 0: cmd_dir |= PelcoD.CMD_TILT_UP
        if tilt_speed < 0: cmd_dir |= PelcoD.CMD_TILT_DOWN
        
        abs_pan_speed = abs(pan_speed)
        abs_tilt_speed = abs(tilt_speed)
        
        packet = PelcoD.make_command(self._device_id, 0x00, cmd_dir, abs_pan_speed, abs_tilt_speed)
        self._send_packet(packet, "Move")

    def stop(self):
        packet = PelcoD.make_command(self._device_id, 0x00, PelcoD.CMD_STOP, 0x00, 0x00)
        self._send_packet(packet, "STOP")

        if self._is_abs_moving:
            self._stop_abs_monitoring()

        if self._is_connected:
            # Query final positions after stopping (delayed to allow deceleration)
            QTimer.singleShot(200, self.query_both_positions)
            
            # Restart auto-query timer if enabled (delayed to prevent Tx/Rx collisions)
            if self._auto_query_enabled:
                QTimer.singleShot(400, lambda: self._query_timer.start(self._auto_query_interval) if self.is_connected() else None)

    def go_to_absolute_position(self, pan_angle: Optional[float], tilt_angle: Optional[float], limits: dict):
        if self._query_timer.isActive():
            self._query_timer.stop()

        if self._is_abs_moving:
            self._stop_abs_monitoring()

        self._is_abs_moving = True
        self._abs_target_pan = pan_angle
        self._abs_target_tilt = tilt_angle
        self._abs_last_pan = None
        self._abs_last_tilt = None
        self._abs_pan_no_change_count = 0 if pan_angle is not None else 3
        self._abs_tilt_no_change_count = 0 if tilt_angle is not None else 3
        self._abs_start_time = QDateTime.currentMSecsSinceEpoch()

        if pan_angle is not None:
            pan_packet = PelcoD.make_absolute_move(self._device_id, "PAN", pan_angle, limits['pan'])
            self._send_packet(pan_packet, f"Abs Move Pan to {pan_angle}°")

        if tilt_angle is not None:
            if pan_angle is not None:
                QTimer.singleShot(50, lambda: self._send_delayed_tilt(tilt_angle, limits['tilt']))
            else:
                self._send_delayed_tilt(tilt_angle, limits['tilt'])
        else:
            if self._is_connected:
                self._abs_timer.start(250)

    def _send_delayed_tilt(self, tilt_angle: float, limits: tuple):
        tilt_packet = PelcoD.make_absolute_move(self._device_id, "TILT", tilt_angle, limits)
        self._send_packet(tilt_packet, f"Abs Move Tilt to {tilt_angle}°")

        if self._is_connected:
            self._abs_timer.start(250)

    def _abs_monitor_tick(self):
        if not self.is_connected():
            self._stop_abs_monitoring()
            return

        self.query_both_positions()

        # Safety timeout: stop monitoring after 20 seconds
        elapsed = QDateTime.currentMSecsSinceEpoch() - self._abs_start_time
        if elapsed > 20000:
            self.log_message.emit("SYSTEM", "Absolute move monitoring timed out (20s)")
            self._stop_abs_monitoring()

    def _stop_abs_monitoring(self):
        self._is_abs_moving = False
        self._abs_timer.stop()
        if self._is_connected and self._auto_query_enabled:
            QTimer.singleShot(200, lambda: self._query_timer.start(self._auto_query_interval) if self.is_connected() else None)

    def _check_abs_arrival(self, q_type: str, angle: float):
        elapsed = QDateTime.currentMSecsSinceEpoch() - self._abs_start_time

        if q_type == "PAN":
            target = self._abs_target_pan
            if target is None:
                return
            is_close = abs(angle - target) < 0.1
            if self._abs_last_pan is not None:
                diff = abs(angle - self._abs_last_pan)
                if diff < 0.005:
                    if elapsed > 1000 or is_close:
                        self._abs_pan_no_change_count += 1
                else:
                    self._abs_pan_no_change_count = 0
            self._abs_last_pan = angle

        elif q_type == "TILT":
            target = self._abs_target_tilt
            if target is None:
                return
            is_close = abs(angle - target) < 0.1
            if self._abs_last_tilt is not None:
                diff = abs(angle - self._abs_last_tilt)
                if diff < 0.005:
                    if elapsed > 1000 or is_close:
                        self._abs_tilt_no_change_count += 1
                else:
                    self._abs_tilt_no_change_count = 0
            self._abs_last_tilt = angle

        if self._abs_pan_no_change_count >= 3 and self._abs_tilt_no_change_count >= 3:
            msg_parts = []
            if self._abs_target_pan is not None:
                msg_parts.append(f"Pan {self._pan_angle:.2f}°")
            if self._abs_target_tilt is not None:
                msg_parts.append(f"Tilt {self._tilt_angle:.2f}°")
            msg = ", ".join(msg_parts)
            self.log_message.emit("SYSTEM", f"Arrived at target position: {msg}")
            self.absolute_move_finished.emit(self._pan_angle, self._tilt_angle)
            self._stop_abs_monitoring()

    def set_position_with_speed(self, pan_angle: float, pan_speed: int, tilt_angle: float, tilt_speed: int, limits: dict):
        if self._query_timer.isActive():
            self._query_timer.stop()

        pan_speed = max(0x01, min(pan_speed, 0x3F))
        tilt_speed = max(0x01, min(tilt_speed, 0x3F))
        speed_packet = PelcoD.make_command(self._device_id, 0x00, PelcoD.SET_PRESET_SPEED, pan_speed, tilt_speed)
        self._send_packet(speed_packet, f"Set Speed Pan:{pan_speed}, Tilt:{tilt_speed}")

        QTimer.singleShot(50, lambda: self.go_to_absolute_position(pan_angle, tilt_angle, limits))
        
    def query_position(self, single_query: Optional[str] = None):
        if single_query == 'pan':
            packet = PelcoD.make_command(self._device_id, 0x00, PelcoD.QUERY_PAN, 0x00, 0x00)
            self._send_packet(packet, "Query Pan")
        elif single_query == 'tilt':
            packet = PelcoD.make_command(self._device_id, 0x00, PelcoD.QUERY_TILT, 0x00, 0x00)
            self._send_packet(packet, "Query Tilt")
        else: # Auto-query tick
            if self._query_toggle:
                self.query_position('pan')
            else:
                self.query_position('tilt')
            self._query_toggle = not self._query_toggle

    def query_both_positions(self):
        if self.is_connected():
            self.query_position('pan')
            QTimer.singleShot(100, lambda: self.query_position('tilt'))

    def start_auto_query(self, interval_ms: int):
        self._auto_query_enabled = True
        self._auto_query_interval = interval_ms
        if self.is_connected():
            self._query_timer.start(interval_ms)
            self.log_message.emit("SYSTEM", "Auto-Query Enabled")

    def stop_auto_query(self):
        self._auto_query_enabled = False
        self._query_timer.stop()
        if self.is_connected():
            self.log_message.emit("SYSTEM", "Auto-Query Disabled")

    def get_current_pan(self) -> float: return self._pan_angle
    def get_current_tilt(self) -> float: return self._tilt_angle
    
    def _read_serial_data(self):
        buffer = self._serial.readAll()
        if not buffer: return

        self._rx_buffer.append(buffer)

        while self._rx_buffer.length() >= 7:
            start_index = self._rx_buffer.indexOf(b'\xFF')
            if start_index == -1:
                self._rx_buffer.clear()
                return
            if start_index > 0:
                self._rx_buffer = self._rx_buffer.mid(start_index)
            if self._rx_buffer.length() < 7:
                return

            packet = self._rx_buffer.left(7)
            packet_data = packet.data()
            
            # Log the complete 7-byte packet
            hex_str = " ".join(f"{b:02X}" for b in packet_data)
            self.log_message.emit("Rx", hex_str)

            q_type, angle = PelcoD.parse_response(packet_data)
            if q_type:
                if q_type == "PAN": self._pan_angle = angle
                elif q_type == "TILT": self._tilt_angle = angle
                self.position_updated.emit(q_type, angle)
                if self._is_abs_moving:
                    self._check_abs_arrival(q_type, angle)
            self._rx_buffer = self._rx_buffer.mid(7)

class PTUControllerApp(QMainWindow):
    SETTINGS_FILE = "settings.json"
    PAN_LIMITS = (0.0, 359.99)
    TILT_LIMITS = (-60.0, 60.0)
    QUERY_INTERVAL_MS = 150

    def __init__(self):
        super().__init__()
        self.presets = [{"pan": None, "tilt": None} for _ in range(6)]
        self.manager = PTUManager(self)
        self.init_ui()
        self.apply_dark_theme()
        
        self.manager.connection_changed.connect(self.on_connection_changed)
        self.manager.position_updated.connect(self.on_position_updated)
        self.manager.log_message.connect(self.on_log_message)
        self.manager.absolute_move_finished.connect(self.on_absolute_move_finished)
        
        self.refresh_ports()
        self.load_settings()
        self.on_connection_changed(False)

    def init_ui(self):
        self.setWindowTitle("BIT-PT510 PTU Controller (Refactored)")
        self.resize(1100, 720)
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.addLayout(self._create_left_layout(), stretch=4)
        main_layout.addLayout(self._create_right_layout(), stretch=6)
        self.setStatusBar(QStatusBar())

    def _create_left_layout(self) -> QVBoxLayout:
        layout = QVBoxLayout()
        
        # Logo & Title Header
        logo_layout = QHBoxLayout()
        logo_label = QLabel()
        pixmap = QPixmap("logo.png")
        if not pixmap.isNull():
            logo_label.setPixmap(pixmap.scaledToHeight(35, Qt.TransformationMode.SmoothTransformation))
        else:
            logo_label.setText("IAE")
            logo_label.setStyleSheet("font-size: 20px; font-weight: bold; color: #62b4ff; font-family: 'Segoe UI';")
        logo_layout.addWidget(logo_label)
        
        title_label = QLabel("BIT-PT510 PTU Controller")
        title_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #dfe1e5; margin-left: 10px;")
        logo_layout.addWidget(title_label)
        logo_layout.addStretch()
        
        layout.addLayout(logo_layout)
        layout.addSpacing(5)
        
        layout.addWidget(self._create_connection_group())
        layout.addWidget(self._create_manual_control_group())
        layout.addWidget(self._create_preset_group())
        layout.addStretch()
        return layout

    def _create_right_layout(self) -> QVBoxLayout:
        layout = QVBoxLayout()
        layout.addWidget(self._create_monitoring_group())
        layout.addWidget(self._create_absolute_pos_group())
        layout.addWidget(self._create_log_group())
        return layout

    def _create_connection_group(self) -> QGroupBox:
        group = QGroupBox("CONNECTION")
        grid = QGridLayout(group)
        self.cb_port = QComboBox()
        self.cb_baud = QComboBox()
        self.cb_baud.addItems(["2400", "4800", "9600", "19200"])
        self.le_id = QLineEdit()
        self.le_id.setMaxLength(2)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connection)
        self.lbl_status = QLabel("Disconnected")
        self.lbl_status.setStyleSheet("color: #ff5555; font-weight: bold;")
        grid.addWidget(QLabel("COM Port:"), 0, 0); grid.addWidget(self.cb_port, 0, 1)
        grid.addWidget(QLabel("Baud Rate:"), 0, 2); grid.addWidget(self.cb_baud, 0, 3)
        grid.addWidget(QLabel("Device ID (Hex):"), 1, 0); grid.addWidget(self.le_id, 1, 1)
        grid.addWidget(self.btn_connect, 1, 2); grid.addWidget(self.lbl_status, 1, 3)
        return group

    def _create_manual_control_group(self) -> QGroupBox:
        self.grp_manual = QGroupBox("MANUAL CONTROL")
        main_layout = QHBoxLayout(self.grp_manual)
        dir_grid = QGridLayout()
        directions = {
            "↖": (-1, 1), "↑\nUp": (0, 1), "↗": (1, 1),
            "←\nLeft": (-1, 0), "STOP": (0, 0), "→\nRight": (1, 0),
            "↙": (-1, -1), "↓\nDown": (0, -1), "↘": (1, -1)
        }
        
        for text, (p_speed_mul, t_speed_mul) in directions.items():
            r, c = -t_speed_mul + 1, p_speed_mul + 1
            btn = QPushButton(text)
            btn.setFixedSize(70, 70)
            if text == "STOP":
                btn.setStyleSheet("background-color: #d9534f; color: white; font-weight: bold;")
                btn.clicked.connect(self.manager.stop)
            else:
                btn.pressed.connect(lambda p=p_speed_mul, t=t_speed_mul: self.handle_move_press(p, t))
                btn.released.connect(self.manager.stop)
            dir_grid.addWidget(btn, r, c)
        main_layout.addLayout(dir_grid)

        speed_sliders_layout = QHBoxLayout()
        
        # Pan Speed Layout (Label -> Slider -> Value LCD)
        pan_speed_layout = QVBoxLayout()
        lbl_pan_title = QLabel("Pan Speed")
        lbl_pan_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        pan_speed_layout.addWidget(lbl_pan_title)
        
        self.slider_pan_speed = QSlider(Qt.Orientation.Vertical)
        self.slider_pan_speed.setRange(0x00, 0x3F)
        self.slider_pan_speed.setValue(0x20)
        self.slider_pan_speed.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.slider_pan_speed.setFixedWidth(30)
        pan_speed_layout.addWidget(self.slider_pan_speed, alignment=Qt.AlignmentFlag.AlignHCenter)
        
        self.lcd_pan_speed = QLCDNumber()
        self.lcd_pan_speed.setDigitCount(2)
        self.lcd_pan_speed.setSegmentStyle(QLCDNumber.SegmentStyle.Flat)
        self.lcd_pan_speed.setFixedSize(55, 35)
        self.lcd_pan_speed.setStyleSheet("QLCDNumber { min-height: 0px; background-color: #1e1f22; border: 1px solid #393b40; color: #62b4ff; }")
        self.lcd_pan_speed.display(32)
        self.slider_pan_speed.valueChanged.connect(self.lcd_pan_speed.display)
        pan_speed_layout.addWidget(self.lcd_pan_speed, alignment=Qt.AlignmentFlag.AlignHCenter)
        speed_sliders_layout.addLayout(pan_speed_layout)
        
        # Tilt Speed Layout (Label -> Slider -> Value LCD)
        tilt_speed_layout = QVBoxLayout()
        lbl_tilt_title = QLabel("Tilt Speed")
        lbl_tilt_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        tilt_speed_layout.addWidget(lbl_tilt_title)
        
        self.slider_tilt_speed = QSlider(Qt.Orientation.Vertical)
        self.slider_tilt_speed.setRange(0x00, 0x3F)
        self.slider_tilt_speed.setValue(0x20)
        self.slider_tilt_speed.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.slider_tilt_speed.setFixedWidth(30)
        tilt_speed_layout.addWidget(self.slider_tilt_speed, alignment=Qt.AlignmentFlag.AlignHCenter)
        
        self.lcd_tilt_speed = QLCDNumber()
        self.lcd_tilt_speed.setDigitCount(2)
        self.lcd_tilt_speed.setSegmentStyle(QLCDNumber.SegmentStyle.Flat)
        self.lcd_tilt_speed.setFixedSize(55, 35)
        self.lcd_tilt_speed.setStyleSheet("QLCDNumber { min-height: 0px; background-color: #1e1f22; border: 1px solid #393b40; color: #62b4ff; }")
        self.lcd_tilt_speed.display(32)
        self.slider_tilt_speed.valueChanged.connect(self.lcd_tilt_speed.display)
        tilt_speed_layout.addWidget(self.lcd_tilt_speed, alignment=Qt.AlignmentFlag.AlignHCenter)
        speed_sliders_layout.addLayout(tilt_speed_layout)
        
        main_layout.addLayout(speed_sliders_layout)
        return self.grp_manual

    def _create_monitoring_group(self) -> QGroupBox:
        self.grp_monitoring = QGroupBox("MONITORING")
        vbox = QVBoxLayout(self.grp_monitoring)
        
        display_hbox = QHBoxLayout()
        self.lcd_pan = QLCDNumber()
        self.lcd_tilt = QLCDNumber()
        for lcd in [self.lcd_pan, self.lcd_tilt]:
            lcd.setDigitCount(6)
            lcd.setSegmentStyle(QLCDNumber.SegmentStyle.Flat)
        display_hbox.addWidget(QLabel("Pan:"))
        display_hbox.addWidget(self.lcd_pan)
        display_hbox.addWidget(QLabel("°"))
        display_hbox.addSpacing(20)
        display_hbox.addWidget(QLabel("Tilt:"))
        display_hbox.addWidget(self.lcd_tilt)
        display_hbox.addWidget(QLabel("°"))
        vbox.addLayout(display_hbox)
        
        opt_hbox = QHBoxLayout()
        self.chk_auto = QCheckBox("Enable Auto-Query")
        self.chk_auto.toggled.connect(self.toggle_auto_query)
        self.btn_query_now = QPushButton("Query Now")
        self.btn_query_now.clicked.connect(self.manager.query_both_positions)
        opt_hbox.addWidget(self.chk_auto)
        opt_hbox.addWidget(self.btn_query_now)
        vbox.addLayout(opt_hbox)
        return self.grp_monitoring

    def _create_absolute_pos_group(self) -> QGroupBox:
        self.grp_absolute = QGroupBox("ABSOLUTE POSITION")
        vbox = QVBoxLayout(self.grp_absolute)
        
        grid = QGridLayout()
        self.spn_pan = QDoubleSpinBox(); self.spn_pan.setRange(*self.PAN_LIMITS)
        self.spn_tilt = QDoubleSpinBox(); self.spn_tilt.setRange(*self.TILT_LIMITS)
        for spn in [self.spn_pan, self.spn_tilt]: spn.setDecimals(2)
        
        large_font = QFont("Segoe UI", 13)
        lbl_pan = QLabel("Pan (°)(0~359.99):"); lbl_pan.setFont(large_font)
        lbl_tilt = QLabel("Tilt (°)(-60~60):"); lbl_tilt.setFont(large_font)
        self.spn_pan.setFont(large_font)
        self.spn_tilt.setFont(large_font)

        self.btn_pan_go = QPushButton("Go")
        self.btn_pan_go.setObjectName("btn_go")
        self.btn_pan_go.clicked.connect(self.handle_pan_abs_move)

        self.btn_tilt_go = QPushButton("Go")
        self.btn_tilt_go.setObjectName("btn_go")
        self.btn_tilt_go.clicked.connect(self.handle_tilt_abs_move)
        
        grid.addWidget(lbl_pan, 0, 0)
        grid.addWidget(self.spn_pan, 0, 1)
        grid.addWidget(self.btn_pan_go, 0, 2)
        grid.addWidget(lbl_tilt, 1, 0)
        grid.addWidget(self.spn_tilt, 1, 1)
        grid.addWidget(self.btn_tilt_go, 1, 2)
        vbox.addLayout(grid)
        
        btn_hbox = QHBoxLayout()
        self.btn_abs_move = QPushButton("Go (Both)")
        self.btn_abs_move.setObjectName("btn_go")
        self.btn_abs_move.clicked.connect(self.handle_abs_move)
        self.btn_home = QPushButton("Home")
        self.btn_home.setObjectName("btn_home")
        self.btn_home.clicked.connect(self.handle_home_move)
        
        btn_hbox.addWidget(self.btn_abs_move)
        btn_hbox.addWidget(self.btn_home)
        vbox.addLayout(btn_hbox)
        
        return self.grp_absolute

    def _create_preset_group(self) -> QGroupBox:
        self.grp_presets = QGroupBox("PRESET POSITIONS")
        layout = QGridLayout(self.grp_presets)
        
        self.preset_labels = []
        self.preset_go_btns = []
        self.preset_save_btns = []
        
        for i in range(6):
            lbl_name = QLabel(f"P{i+1}:")
            lbl_name.setStyleSheet("font-weight: bold; color: #bbbbbb;")
            lbl_val = QLabel("Not Saved")
            lbl_val.setStyleSheet("color: #888888; font-family: 'Consolas';")
            self.preset_labels.append(lbl_val)
            
            btn_go = QPushButton("Go")
            btn_go.setStyleSheet("""
                QPushButton { background-color: #2b78c5; border: 1px solid #1e5c9b; color: white; padding: 4px; min-width: 45px; font-weight: bold; }
                QPushButton:hover { background-color: #4392e0; }
                QPushButton:pressed { background-color: #1a4f86; }
                QPushButton:disabled { background-color: #3a3f46; border: 1px solid #2d3135; color: #7a7e85; }
            """)
            btn_go.clicked.connect(lambda checked, idx=i: self.go_to_preset(idx))
            self.preset_go_btns.append(btn_go)
            
            btn_save = QPushButton("Save")
            btn_save.setStyleSheet("""
                QPushButton { background-color: #10b981; border: 1px solid #059669; color: white; padding: 4px; min-width: 45px; font-weight: bold; }
                QPushButton:hover { background-color: #34d399; }
                QPushButton:pressed { background-color: #047857; }
                QPushButton:disabled { background-color: #3a3f46; border: 1px solid #2d3135; color: #7a7e85; }
            """)
            btn_save.clicked.connect(lambda checked, idx=i: self.save_preset(idx))
            self.preset_save_btns.append(btn_save)
            
            layout.addWidget(lbl_name, i, 0)
            layout.addWidget(lbl_val, i, 1)
            layout.addWidget(btn_go, i, 2)
            layout.addWidget(btn_save, i, 3)
            
        return self.grp_presets

    def _create_log_group(self) -> QGroupBox:
        group = QGroupBox("COMMUNICATION LOG")
        vbox = QVBoxLayout(group)
        self.txt_log = QTextEdit(); self.txt_log.setReadOnly(True)
        self.txt_log.setFont(QFont("Consolas", 10))
        vbox.addWidget(self.txt_log)
        btn_clear_log = QPushButton("Clear Log")
        btn_clear_log.clicked.connect(self.txt_log.clear)
        vbox.addWidget(btn_clear_log)
        return group
    
    def toggle_connection(self):
        if self.manager.is_connected():
            self.manager.disconnect()
        else:
            port = self.cb_port.currentText()
            if not port:
                self.statusBar().showMessage("연결할 COM 포트를 찾을 수 없습니다.", 5000)
                return
            try:
                device_id = int(self.le_id.text(), 16)
                self.manager.connect(port, int(self.cb_baud.currentText()), device_id)
            except ValueError:
                self.statusBar().showMessage("잘못된 ID 형식입니다. 16진수 두 자리(예: 01)로 입력하세요.", 5000)

    def handle_move_press(self, p_speed_mul: int, t_speed_mul: int):
        pan_speed = self.slider_pan_speed.value() * p_speed_mul
        tilt_speed = self.slider_tilt_speed.value() * t_speed_mul
        self.manager.move(pan_speed, tilt_speed)

    def handle_pan_abs_move(self):
        pan = self.spn_pan.value()
        self.manager.go_to_absolute_position(pan, None, {'pan': self.PAN_LIMITS, 'tilt': self.TILT_LIMITS})

    def handle_tilt_abs_move(self):
        tilt = self.spn_tilt.value()
        self.manager.go_to_absolute_position(None, tilt, {'pan': self.PAN_LIMITS, 'tilt': self.TILT_LIMITS})

    def handle_abs_move(self):
        pan = self.spn_pan.value()
        tilt = self.spn_tilt.value()
        self.manager.go_to_absolute_position(pan, tilt, {'pan': self.PAN_LIMITS, 'tilt': self.TILT_LIMITS})
        
    def handle_home_move(self):
        self.spn_pan.setValue(0.0)
        self.spn_tilt.setValue(0.0)
        self.handle_abs_move()

    def toggle_auto_query(self, checked: bool):
        if checked: self.manager.start_auto_query(self.QUERY_INTERVAL_MS)
        else: self.manager.stop_auto_query()

    def on_connection_changed(self, is_connected: bool):
        self.grp_manual.setEnabled(is_connected)
        self.grp_absolute.setEnabled(is_connected)
        self.grp_presets.setEnabled(is_connected)
        self.chk_auto.setEnabled(is_connected)
        self.btn_query_now.setEnabled(is_connected)
        
        self.update_preset_labels()
        
        if is_connected:
            self.lbl_status.setText("Connected"); self.lbl_status.setStyleSheet("color: #55ff55; font-weight: bold;")
            self.btn_connect.setText("Disconnect"); self.statusBar().showMessage("장치에 연결되었습니다.", 3000)
        else:
            self.lbl_status.setText("Disconnected"); self.lbl_status.setStyleSheet("color: #ff5555; font-weight: bold;")
            self.btn_connect.setText("Connect"); self.statusBar().showMessage("장치 연결이 해제되었습니다.", 3000)
            self.chk_auto.setChecked(False)

    def update_preset_labels(self):
        for i, preset in enumerate(self.presets):
            pan = preset.get("pan")
            tilt = preset.get("tilt")
            if pan is not None and tilt is not None:
                self.preset_labels[i].setText(f"{pan:.2f}°, {tilt:.2f}°")
                self.preset_labels[i].setStyleSheet("color: #62b4ff; font-family: 'Consolas'; font-weight: bold;")
                self.preset_go_btns[i].setEnabled(self.manager.is_connected())
            else:
                self.preset_labels[i].setText("Not Saved")
                self.preset_labels[i].setStyleSheet("color: #888888; font-family: 'Consolas';")
                self.preset_go_btns[i].setEnabled(False)

    def save_preset(self, idx: int):
        pan = self.manager.pan_angle
        tilt = self.manager.tilt_angle
        self.presets[idx] = {"pan": pan, "tilt": tilt}
        self.update_preset_labels()
        self.save_settings()
        self.statusBar().showMessage(f"Preset {idx+1}에 현재 위치({pan:.2f}°, {tilt:.2f}°)를 저장했습니다.", 3000)

    def go_to_preset(self, idx: int):
        preset = self.presets[idx]
        pan = preset.get("pan")
        tilt = preset.get("tilt")
        if pan is not None and tilt is not None:
            self.spn_pan.setValue(pan)
            self.spn_tilt.setValue(tilt)
            pan_speed = self.slider_pan_speed.value()
            tilt_speed = self.slider_tilt_speed.value()
            self.manager.set_position_with_speed(pan, pan_speed, tilt, tilt_speed, {'pan': self.PAN_LIMITS, 'tilt': self.TILT_LIMITS})
            self.statusBar().showMessage(f"Preset {idx+1} 위치로 이동 중...", 3000)

    def on_position_updated(self, q_type: str, angle: float):
        if q_type == "PAN": self.lcd_pan.display(f"{angle:.2f}")
        elif q_type == "TILT": self.lcd_tilt.display(f"{angle:.2f}")

    def on_absolute_move_finished(self, pan: float, tilt: float):
        self.statusBar().showMessage(f"목표 위치에 도착했습니다. (Pan: {pan:.2f}°, Tilt: {tilt:.2f}°)", 5000)

    def on_log_message(self, direction: str, msg: str):
        time_str = QDateTime.currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz")
        color_map = {"Tx": "#62b4ff", "Rx": "#55ff55", "ERROR": "#d9534f"}
        color = color_map.get(direction, "#bbbbbb")
        html = f"<span style='color:#888888;'>[{time_str}]</span> <span style='color:{color}; font-weight:bold;'>[{direction}]</span> {msg}"
        self.txt_log.append(html)
        
    def refresh_ports(self):
        self.cb_port.clear()
        self.cb_port.addItems(self.manager.get_available_ports())

    def load_settings(self):
        try:
            with open(self.SETTINGS_FILE, 'r') as f:
                settings = json.load(f)
            self.cb_port.setCurrentText(settings.get("port", ""))
            self.cb_baud.setCurrentText(str(settings.get("baud", "9600")))
            self.le_id.setText(settings.get("id", "01"))
            
            loaded_presets = settings.get("presets", [])
            while len(loaded_presets) < 6:
                loaded_presets.append({"pan": None, "tilt": None})
            self.presets = loaded_presets[:6]
            
            self.update_preset_labels()
            self.statusBar().showMessage("이전 설정을 불러왔습니다.", 3000)
        except (FileNotFoundError, json.JSONDecodeError):
            self.presets = [{"pan": None, "tilt": None} for _ in range(6)]
            self.update_preset_labels()
            self.statusBar().showMessage("설정 파일이 없거나 손상되어 기본값으로 시작합니다.", 3000)
            self.cb_baud.setCurrentText("9600")
            self.le_id.setText("01")

    def save_settings(self):
        settings = { 
            "port": self.cb_port.currentText(), 
            "baud": self.cb_baud.currentText(), 
            "id": self.le_id.text(),
            "presets": self.presets
        }
        try:
            with open(self.SETTINGS_FILE, 'w') as f:
                json.dump(settings, f, indent=4)
        except IOError as e:
            self.statusBar().showMessage(f"설정 파일 저장 실패: {e}", 5000)

    def closeEvent(self, event):
        self.save_settings()
        self.manager.disconnect()
        super().closeEvent(event)
        
    def apply_dark_theme(self):
        dark_stylesheet = """
            QWidget { background-color: #2b2d30; color: #dfe1e5; font-family: 'Segoe UI', Arial; }
            QGroupBox { font-weight: bold; border: 2px solid #393b40; border-radius: 6px; margin-top: 12px; padding: 10px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }
            QPushButton { background-color: #393b40; border: 1px solid #4e5157; border-radius: 4px; padding: 6px; min-height: 20px; }
            QPushButton:hover { background-color: #4e5157; }
            QPushButton:pressed { background-color: #1e1f22; }
            QPushButton#btn_go { background-color: #2b78c5; border: 1px solid #1e5c9b; color: white; font-weight: bold; }
            QPushButton#btn_go:hover { background-color: #4392e0; }
            QPushButton#btn_go:pressed { background-color: #1a4f86; }
            QPushButton#btn_go:disabled { background-color: #3a3f46; border: 1px solid #2d3135; color: #7a7e85; }
            QPushButton#btn_home { background-color: #d47a2a; border: 1px solid #a85f1e; color: white; font-weight: bold; }
            QPushButton#btn_home:hover { background-color: #e89143; }
            QPushButton#btn_home:pressed { background-color: #a05a1d; }
            QPushButton#btn_home:disabled { background-color: #3a3f46; border: 1px solid #2d3135; color: #7a7e85; }
            QComboBox, QLineEdit { background-color: #1e1f22; border: 1px solid #4e5157; border-radius: 4px; padding: 4px; color: white; }
            QDoubleSpinBox { background-color: #1e1f22; border: 1px solid #4e5157; border-radius: 4px; padding: 4px; padding-right: 20px; color: white; }
            QDoubleSpinBox::up-button { subcontrol-origin: border; subcontrol-position: top right; width: 18px; border-left: 1px solid #4e5157; background-color: #393b40; border-top-right-radius: 4px; }
            QDoubleSpinBox::up-button:hover { background-color: #4e5157; }
            QDoubleSpinBox::up-arrow { width: 0; height: 0; border-left: 4px solid transparent; border-right: 4px solid transparent; border-bottom: 5px solid #dfe1e5; }
            QDoubleSpinBox::down-button { subcontrol-origin: border; subcontrol-position: bottom right; width: 18px; border-left: 1px solid #4e5157; background-color: #393b40; border-bottom-right-radius: 4px; }
            QDoubleSpinBox::down-button:hover { background-color: #4e5157; }
            QDoubleSpinBox::down-arrow { width: 0; height: 0; border-left: 4px solid transparent; border-right: 4px solid transparent; border-top: 5px solid #dfe1e5; }
            QLCDNumber { background-color: #1e1f22; border: 1px solid #393b40; color: #62b4ff; min-height: 50px; }
            QTextEdit { background-color: #1e1f22; border: 1px solid #393b40; color: #a9b7c6; }
            QSlider::handle:vertical { background: #62b4ff; height: 15px; border-radius: 3px; }
            QStatusBar { font-weight: bold; }
        """
        self.setStyleSheet(dark_stylesheet)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = PTUControllerApp()
    window.show()
    sys.exit(app.exec())
