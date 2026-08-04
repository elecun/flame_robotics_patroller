import sys
import time
import json
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem,
    QLabel, QHBoxLayout, QPushButton, QMessageBox, QLineEdit, QSlider,
    QDialog, QGroupBox, QFormLayout, QCheckBox, QComboBox, QDoubleSpinBox, QSpinBox
)
from PyQt6.QtCore import QTimer, Qt

try:
    from canlib import canlib, Frame
    CANLIB_AVAILABLE = True
except (ImportError, Exception, BaseException) as e:
    CANLIB_AVAILABLE = False
    canlib = None
    Frame = None

CONFIG_FILE = "can_config.json"

class CANParser:
    """
    CAN 메시지를 파싱하여 사람이 읽을 수 있는 형태로 변환하는 클래스.
    """
    def __init__(self):
        pass

    def parse(self, can_id, data):
        """
        주어진 CAN ID와 데이터에 따라 메시지를 파싱합니다.
        """
        parsed = {}
        # 0x303: 차량 기어, 주행 상태 모드, 차량 속도 요청
        if can_id == 0x303:
            if len(data) >= 4: # 데이터 길이 확인
                vehicle_gear = data[0] & 0x03
                parsed['Vehicle Gear'] = ["P Gear", "D Gear", "N Gear", "R Gear"][vehicle_gear]
                drive_state_mode = data[1] & 0x03
                parsed['Drive_State_Mode'] = ["Remote Control Mode", "Represents the AD Mode",
                                              "Indicates parallel Mode", "Indicates semi-autonomous"][drive_state_mode]
                vcu_speed_req = int.from_bytes(data[2:4], byteorder='little') * 0.1 - 80
                parsed['Vehicle Speed Request (km/h)'] = f"{vcu_speed_req:.1f}"

        # 0x314: 방향각, EPS 제어 상태 (VCU_EPS_Control_Request, Motorola/Big-Endian)
        elif can_id == 0x314:
            if len(data) >= 3: # 데이터 길이 확인
                directional_angle = int.from_bytes(data[1:3], byteorder='big', signed=True)
                parsed['Direction Angle (deg)'] = f"{directional_angle}"
                parsed['eps Control'] = "Works" if data[0] & 0x01 else "Stops"

        # 0x304: 차량 속도, 휠 엔드 각도, 브레이크 압력
        elif can_id == 0x304:
            if len(data) >= 6: # 데이터 길이 확인
                speed = int.from_bytes(data[0:2], byteorder='little') * 0.1 - 80
                parsed['Vehicle Speed (km/h)'] = f"{speed:.1f}"
                steering_raw = int.from_bytes(data[4:6], 'little') & 0x3FF # DBC: Vehicle_Steering_Angle은 10bit
                parsed['Vehicle Wheel End Angle (deg)'] = f"{steering_raw * 0.1 - 35:.1f}"
                parsed['Vehicle Break Pressure (Mps)'] = f"{int.from_bytes(data[2:4], 'little') * 0.01:.2f}"

        # 0x301: 라이트, 스위치 상태
        elif can_id == 0x301:
            if len(data) >= 6: # 데이터 길이 확인
                parsed['Brake Light'] = "ON" if data[5] & 0x01 else "OFF"
                parsed['Head Light'] = "ON" if data[1] & 0x80 else "OFF"
                parsed['Emergency Button'] = "Pressed" if data[0] & 0x01 else "Not Pressed"
                parsed['Back Touch Switch State'] = "trigger" if data[1] & 0x20 else "Not trigger"
                parsed['Front Touch Switch State'] = "trigger" if data[1] & 0x10 else "Not trigger"

        # 0x501~0x506 (AD Control Echo - 버스 상에 수신된 경우)
        elif can_id == 0x501:
            if len(data) >= 1:
                cntr = (data[0] >> 4) & 0x0F
                valid = data[0] & 0x01
                parsed['AD_Control_Request_Flag'] = f"{valid} (cntr={cntr})"
        elif can_id == 0x502:
            if len(data) >= 6:
                cntr = (data[0] >> 4) & 0x0F
                valid = data[0] & 0x01
                angle_raw = int.from_bytes(data[4:6], byteorder='little')
                angle_deg = (angle_raw * 0.1) - 30.0
                parsed['AD_Steering_Angle_Cmd (deg)'] = f"{angle_deg:.1f} (valid={valid}, cntr={cntr})"
        elif can_id == 0x503:
            if len(data) >= 2:
                cntr = (data[0] >> 4) & 0x0F
                valid = data[0] & 0x01
                parsed['AD_Brake_Cmd (%)'] = f"{data[1]}% (valid={valid}, cntr={cntr})"
        elif can_id == 0x504:
            if len(data) >= 8:
                cntr = (data[0] >> 4) & 0x0F
                valid = data[0] & 0x01
                workmode = data[2]
                gear_code = data[3]
                gear_str = ["P", "D", "N", "R"][gear_code] if gear_code < 4 else f"Unknown({gear_code})"
                accde_raw = data[4]
                accde_val = (accde_raw * 0.1) - 5.0
                torque = data[5]
                ad_speed = int.from_bytes(data[6:8], byteorder='little') * 0.1
                parsed['AD_Accelerate'] = f"Valid={valid}, Gear={gear_str}, Speed={ad_speed:.1f}km/h, AccDe={accde_val:.1f}, Torque={torque}%, WorkMode={workmode} (cntr={cntr})"
        elif can_id == 0x506:
            if len(data) >= 2:
                cntr = (data[0] >> 4) & 0x0F
                l_turn = "ON" if data[0] & 0x01 else "OFF"
                r_turn = "ON" if data[0] & 0x02 else "OFF"
                horn = "ON" if data[0] & 0x04 else "OFF"
                head = "ON" if data[0] & 0x08 else "OFF"
                brake_light = "ON" if data[1] & 0x01 else "OFF"
                parsed['AD_Control_Body'] = f"Left={l_turn}, Right={r_turn}, Horn={horn}, Head={head}, BrakeLight={brake_light} (cntr={cntr})"

        return parsed


class ADControlPanel(QDialog):
    """
    데이터시트 CAN ID의 AD_Control_Flag(0x501)/Steering(0x502)/Brake(0x503)/
    Accelerate(0x504)/Body(0x506) 각 신호를 필드 단위로 구성해서 20ms 주기로
    전송하는 패널. 체크박스로 메시지별 전송을 켜고 끌 수 있고, 각 값은
    스펙에 맞는 콤보박스/스핀박스로만 입력받아 바이트 정렬 실수를 방지한다.
    """
    def __init__(self, main_window):
        super().__init__(main_window)
        self.main_window = main_window
        self.setWindowTitle("AD Control 메시지 전송 (0x501~0x506)")
        self.resize(480, 560)

        # 메시지별 롤링 하트비트 카운터 (0~15)
        self.msg_counters = {0x501: 0, 0x502: 0, 0x503: 0, 0x504: 0, 0x506: 0}

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("체크된 메시지만 20ms 주기로 전송됩니다. (하트비트 자동 순환)"))

        # ---- 0x501 AD_Control_Flag ----
        self.chk_501_en = QCheckBox("전송 활성화")
        self.cmb_501_valid = QComboBox()
        self.cmb_501_valid.addItems(["0: Invalid", "1: Valid"])
        self.cmb_501_valid.setCurrentIndex(1)
        layout.addWidget(self._build_group("0x501 AD_Control_Flag", [
            ("전송", self.chk_501_en),
            ("AD_Control_Request_Flag", self.cmb_501_valid),
        ]))

        # ---- 0x502 AD_Control_Steering ----
        self.chk_502_en = QCheckBox("전송 활성화")
        self.cmb_502_valid = QComboBox()
        self.cmb_502_valid.addItems(["0: Invalid", "1: Controllable steering"])
        self.cmb_502_valid.setCurrentIndex(1)
        self.spin_502_angle = QDoubleSpinBox()
        self.spin_502_angle.setRange(-30.0, 30.0)
        self.spin_502_angle.setSingleStep(0.1)
        self.spin_502_angle.setSuffix(" deg")
        layout.addWidget(self._build_group("0x502 AD_Control_Steering", [
            ("전송", self.chk_502_en),
            ("AD_Steering_Valid", self.cmb_502_valid),
            ("AD_Steering_Angle_Cmd (-30~30)", self.spin_502_angle),
        ]))

        # ---- 0x503 AD_Control_Brake ----
        self.chk_503_en = QCheckBox("전송 활성화")
        self.cmb_503_valid = QComboBox()
        self.cmb_503_valid.addItems(["0: Invalid", "1: Controllable brake"])
        self.cmb_503_valid.setCurrentIndex(1)
        self.spin_503_brake = QSpinBox()
        self.spin_503_brake.setRange(0, 100)
        self.spin_503_brake.setSuffix(" %")
        layout.addWidget(self._build_group("0x503 AD_Control_Brake", [
            ("전송", self.chk_503_en),
            ("AD_Brake_Valid", self.cmb_503_valid),
            ("AD_Brake_Cmd", self.spin_503_brake),
        ]))

        # ---- 0x504 AD_Control_Accelerate ----
        self.chk_504_en = QCheckBox("전송 활성화")
        self.cmb_504_valid = QComboBox()
        self.cmb_504_valid.addItems(["0: Invalid", "1: Controllable accelerate"])
        self.cmb_504_valid.setCurrentIndex(1)
        self.cmb_504_workmode = QComboBox()
        self.cmb_504_workmode.addItems(["0: Normal Mode", "1: High Mode", "2: Speed Mode", "3: Torque Mode"])
        self.cmb_504_gear = QComboBox()
        self.cmb_504_gear.addItems(["0: P Gear", "1: D Gear", "2: N Gear", "3: R Gear"])
        self.spin_504_accde = QDoubleSpinBox()
        self.spin_504_accde.setRange(-5.0, 5.0)
        self.spin_504_accde.setSingleStep(0.1)
        self.spin_504_torque = QSpinBox()
        self.spin_504_torque.setRange(0, 100)
        self.spin_504_torque.setSuffix(" %")
        self.spin_504_speed = QDoubleSpinBox()
        self.spin_504_speed.setRange(0.0, 80.0)
        self.spin_504_speed.setSingleStep(0.1)
        self.spin_504_speed.setSuffix(" km/h")
        layout.addWidget(self._build_group("0x504 AD_Control_Accelerate", [
            ("전송", self.chk_504_en),
            ("AD_Accelerate_Valid", self.cmb_504_valid),
            ("AD_Accelerate_Work_Mode", self.cmb_504_workmode),
            ("AD_Accelerate_Gear", self.cmb_504_gear),
            ("AD_Acc_De (-5~5)", self.spin_504_accde),
            ("AD_Torque_Control", self.spin_504_torque),
            ("AD_Speed_Control", self.spin_504_speed),
        ]))

        # ---- 0x506 AD_Control_Body ----
        self.chk_506_en = QCheckBox("전송 활성화")
        self.chk_506_left = QCheckBox("좌회전")
        self.chk_506_right = QCheckBox("우회전")
        self.chk_506_horn = QCheckBox("경적")
        self.chk_506_head = QCheckBox("전조등")
        self.chk_506_brake_light = QCheckBox("브레이크등")
        layout.addWidget(self._build_group("0x506 AD_Control_Body", [
            ("전송", self.chk_506_en),
            ("AD_Left_Turn_Light", self.chk_506_left),
            ("AD_Right_Turn_Light", self.chk_506_right),
            ("AD_Horn_Control", self.chk_506_horn),
            ("AD_HeadLight", self.chk_506_head),
            ("AD_Brake_Light", self.chk_506_brake_light),
        ]))

        self.btn_stop_all = QPushButton("전체 전송 중지")
        self.btn_stop_all.clicked.connect(self._stop_all)
        layout.addWidget(self.btn_stop_all)

        self.sent_display = QLabel("전송된 프레임: (아직 없음)")
        self.sent_display.setStyleSheet("font-family: Consolas, monospace;")
        self.sent_display.setWordWrap(True)
        layout.addWidget(self.sent_display)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(20)

    def _build_group(self, title, rows):
        box = QGroupBox(title)
        form = QFormLayout()
        for label, widget in rows:
            form.addRow(label, widget)
        box.setLayout(form)
        return box

    def _stop_all(self):
        for chk in (self.chk_501_en, self.chk_502_en, self.chk_503_en, self.chk_504_en, self.chk_506_en):
            chk.setChecked(False)

    def _next_cntr(self, can_id):
        cntr = self.msg_counters[can_id]
        self.msg_counters[can_id] = (cntr + 1) % 16
        return cntr

    def _tick(self):
        bus = self.main_window.bus
        if bus is None:
            self.sent_display.setText("전송된 프레임: CAN 버스가 연결되어 있지 않습니다.")
            return

        try:
            self._tick_impl(bus)
        except Exception as e:
            self.timer.stop()
            self.sent_display.setText(f"!!! 전송 실패로 타이머 정지: {e!r}")
            QMessageBox.critical(self, "CAN 전송 오류",
                                  f"프레임 전송 중 오류가 발생해 반복 전송을 멈췄습니다.\n\n{e!r}")

    def _tick_impl(self, bus):
        sent_lines = []

        if self.chk_501_en.isChecked():
            cntr = self._next_cntr(0x501)
            valid = self.cmb_501_valid.currentIndex() & 0x1
            byte0 = (cntr << 4) | valid
            msg = Frame(id_=0x501, data=bytes([byte0, 0, 0, 0, 0, 0, 0, 0]))
            bus.write(msg)
            sent_lines.append(msg)

        if self.chk_502_en.isChecked():
            cntr = self._next_cntr(0x502)
            valid = self.cmb_502_valid.currentIndex() & 0x1
            byte0 = (cntr << 4) | valid
            raw = int(round((self.spin_502_angle.value() + 30.0) / 0.1))
            raw = max(0, min(0xFFFF, raw))
            data = [byte0, 0, 0, 0, raw & 0xFF, (raw >> 8) & 0xFF, 0, 0]
            msg = Frame(id_=0x502, data=bytes(data))
            bus.write(msg)
            sent_lines.append(msg)

        if self.chk_503_en.isChecked():
            cntr = self._next_cntr(0x503)
            valid = self.cmb_503_valid.currentIndex() & 0x1
            byte0 = (cntr << 4) | valid
            data = [byte0, self.spin_503_brake.value(), 0, 0, 0, 0, 0, 0]
            msg = Frame(id_=0x503, data=bytes(data))
            bus.write(msg)
            sent_lines.append(msg)

        if self.chk_504_en.isChecked():
            cntr = self._next_cntr(0x504)
            valid = self.cmb_504_valid.currentIndex() & 0x1
            byte0 = (cntr << 4) | valid
            work_mode = self.cmb_504_workmode.currentIndex()
            gear = self.cmb_504_gear.currentIndex()
            raw_accde = int(round((self.spin_504_accde.value() + 5.0) / 0.1))
            raw_accde = max(0, min(0xFF, raw_accde))
            raw_speed = int(round(self.spin_504_speed.value() / 0.1))
            raw_speed = max(0, min(0xFFFF, raw_speed))
            data = [byte0, 0, work_mode, gear, raw_accde, self.spin_504_torque.value(),
                    raw_speed & 0xFF, (raw_speed >> 8) & 0xFF]
            msg = Frame(id_=0x504, data=bytes(data))
            bus.write(msg)
            sent_lines.append(msg)

        if self.chk_506_en.isChecked():
            cntr = self._next_cntr(0x506)
            left = 1 if self.chk_506_left.isChecked() else 0
            right = 1 if self.chk_506_right.isChecked() else 0
            horn = 1 if self.chk_506_horn.isChecked() else 0
            head = 1 if self.chk_506_head.isChecked() else 0
            brake_light = 1 if self.chk_506_brake_light.isChecked() else 0
            byte0 = (cntr << 4) | left | (right << 1) | (horn << 2) | (head << 3)
            data = [byte0, brake_light, 0, 0, 0, 0, 0, 0]
            msg = Frame(id_=0x506, data=bytes(data))
            bus.write(msg)
            sent_lines.append(msg)

        if sent_lines:
            lines = ["전송된 프레임 (20ms 주기로 계속 갱신됨):"]
            for msg in sent_lines:
                lines.append(f"  0x{msg.id:03X}: " + " ".join(f"{b:02X}" for b in msg.data))
            self.sent_display.setText("\n".join(lines))
        else:
            self.sent_display.setText("전송된 프레임: 체크된 메시지가 없습니다. 위에서 '전송 활성화'를 체크하세요.")

    def closeEvent(self, event):
        self.timer.stop()
        super().closeEvent(event)


class MainWindow(QMainWindow):
    """
    CAN 통신 모니터링 및 제어를 위한 메인 GUI 창 클래스.
    """
    SPEED_SLIDER_FACTOR = 10.0
    ANGLE_SLIDER_FACTOR = 10.0

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Withus CAN Monitor")
        self.resize(1200, 800)
        self.channel_num = 0  # Kvaser CANlib channel 0 지정
        self.parser = CANParser()

        self._setup_ui()
        self._connect_signals_slots()

        self.bus = None
        self.read_timer = QTimer()
        self.read_timer.timeout.connect(self._read_can_messages)

        self.drive_timer = QTimer()
        self.drive_timer.timeout.connect(self._send_repeated_drive_command)
        self.drive_timer.setInterval(20)

        self.current_speed = 0.0
        self.current_angular = 0.0
        self.ad_msg_counter = 0
        self.ad_mode_requested = False
        self.ad_control_panel = None

        self._update_speed_input_from_slider(self.speed_slider.value())
        self._update_angle_input_from_slider(self.angle_slider.value())

    def _setup_ui(self):
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        left_layout = QVBoxLayout()
        right_layout = QVBoxLayout()

        conn_group = QGroupBox("CAN 버스 연결 설정 (Kvaser CANlib Channel 0)")
        conn_layout = QHBoxLayout()
        self.btn_connect = QPushButton("CAN 연결")
        self.btn_disconnect = QPushButton("CAN 해제")
        conn_layout.addWidget(self.btn_connect)
        conn_layout.addWidget(self.btn_disconnect)
        conn_group.setLayout(conn_layout)
        left_layout.addWidget(conn_group)

        ctrl_group = QGroupBox("차량 제어 슬라이더")
        ctrl_layout = QFormLayout()

        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(int(-30.0 * self.SPEED_SLIDER_FACTOR), int(30.0 * self.SPEED_SLIDER_FACTOR))
        self.speed_slider.setValue(0)
        self.speed_input = QLineEdit("0.0")
        self.speed_input.setFixedWidth(60)

        speed_h_layout = QHBoxLayout()
        speed_h_layout.addWidget(self.speed_slider)
        speed_h_layout.addWidget(self.speed_input)
        ctrl_layout.addRow("목표 속도 (km/h):", speed_h_layout)

        self.angle_slider = QSlider(Qt.Orientation.Horizontal)
        self.angle_slider.setRange(int(-30.0 * self.ANGLE_SLIDER_FACTOR), int(30.0 * self.ANGLE_SLIDER_FACTOR))
        self.angle_slider.setValue(0)
        self.angle_input = QLineEdit("0.0")
        self.angle_input.setFixedWidth(60)

        angle_h_layout = QHBoxLayout()
        angle_h_layout.addWidget(self.angle_slider)
        angle_h_layout.addWidget(self.angle_input)
        ctrl_layout.addRow("조향 각도 (deg):", angle_h_layout)

        ctrl_group.setLayout(ctrl_layout)
        left_layout.addWidget(ctrl_group)

        action_group = QGroupBox("제어 명령")
        action_layout = QVBoxLayout()
        self.btn_stop = QPushButton("차량 정지 (속도/각도 0)")
        self.btn_stop.setStyleSheet("background-color: #ff4d4d; color: white; font-weight: bold;")
        action_layout.addWidget(self.btn_stop)

        self.btn_request_ad_mode = QPushButton("AD Mode 전환 요청 (0x501 Valid=1)")
        self.btn_request_ad_mode.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        action_layout.addWidget(self.btn_request_ad_mode)

        self.btn_request_remote_mode = QPushButton("Remote Control 모드 해제 요청 (0x501 Valid=0)")
        self.btn_request_remote_mode.setStyleSheet("background-color: #ff9800; color: white; font-weight: bold;")
        action_layout.addWidget(self.btn_request_remote_mode)

        self.mode_request_label = QLabel("현재 요청 모드: Remote Control")
        self.mode_request_label.setStyleSheet("font-weight: bold; font-size: 13px;")
        action_layout.addWidget(self.mode_request_label)

        self.btn_open_ad_panel = QPushButton("AD Control 메시지 패널 열기 (0x501~0x506)")
        action_layout.addWidget(self.btn_open_ad_panel)

        self.sent_frames_label = QLabel("전송된 프레임: (아직 없음)")
        self.sent_frames_label.setStyleSheet("font-family: Consolas, monospace;")
        self.sent_frames_label.setWordWrap(True)
        action_layout.addWidget(self.sent_frames_label)

        action_group.setLayout(action_layout)
        left_layout.addWidget(action_group)

        manual_group = QGroupBox("수동 CAN 메시지 전송")
        manual_layout = QFormLayout()

        send1_layout = QHBoxLayout()
        self.input_id = QLineEdit("502")
        self.input_id.setFixedWidth(60)
        self.input_data = [QLineEdit() for _ in range(8)]
        send1_layout.addWidget(QLabel("ID(Hex):"))
        send1_layout.addWidget(self.input_id)
        send1_layout.addWidget(QLabel("Data(Hex):"))
        for field in self.input_data:
            field.setFixedWidth(30)
            send1_layout.addWidget(field)
        self.btn_send1 = QPushButton("전송 1")
        send1_layout.addWidget(self.btn_send1)
        manual_layout.addRow(send1_layout)

        send2_layout = QHBoxLayout()
        self.input_id2 = QLineEdit("504")
        self.input_id2.setFixedWidth(60)
        self.input_data2 = [QLineEdit() for _ in range(8)]
        send2_layout.addWidget(QLabel("ID(Hex):"))
        send2_layout.addWidget(self.input_id2)
        send2_layout.addWidget(QLabel("Data(Hex):"))
        for field in self.input_data2:
            field.setFixedWidth(30)
            send2_layout.addWidget(field)
        self.btn_send2 = QPushButton("전송 2")
        send2_layout.addWidget(self.btn_send2)
        manual_layout.addRow(send2_layout)

        manual_group.setLayout(manual_layout)
        left_layout.addWidget(manual_group)

        table_ctrl_group = QGroupBox("테이블 관리")
        table_ctrl_layout = QHBoxLayout()
        self.btn_clear_tables = QPushButton("테이블 내용 지우기")
        table_ctrl_layout.addWidget(self.btn_clear_tables)
        table_ctrl_group.setLayout(table_ctrl_layout)
        left_layout.addWidget(table_ctrl_group)

        left_layout.addStretch()

        right_layout.addWidget(QLabel("Raw CAN 메시지 (최신 수신)"))
        self.raw_table = QTableWidget(0, 3)
        self.raw_table.setHorizontalHeaderLabels(["ID", "DLC", "Data (Hex)"])
        right_layout.addWidget(self.raw_table)

        right_layout.addWidget(QLabel("Parsed CAN 메시지"))
        self.parsed_table = QTableWidget(0, 2)
        self.parsed_table.setHorizontalHeaderLabels(["Signal Name", "Parsed Value"])
        right_layout.addWidget(self.parsed_table)

        main_layout.addLayout(left_layout, 1)
        main_layout.addLayout(right_layout, 2)

    def _connect_signals_slots(self):
        self.btn_connect.clicked.connect(self.connect_can_interface)
        self.btn_disconnect.clicked.connect(self.disconnect_can_interface)
        self.btn_stop.clicked.connect(self._stop_vehicle)
        self.btn_request_ad_mode.clicked.connect(self._request_ad_mode)
        self.btn_request_remote_mode.clicked.connect(self._request_remote_control_mode)
        self.btn_open_ad_panel.clicked.connect(self._open_ad_control_panel)
        self.btn_clear_tables.clicked.connect(self.clear_tables)
        self.btn_send1.clicked.connect(self._send_can_frame)
        self.btn_send2.clicked.connect(self._send_can_frame2)

        self.speed_slider.valueChanged.connect(self._on_speed_slider_changed)
        self.speed_input.editingFinished.connect(self._on_speed_input_changed)

        self.angle_slider.valueChanged.connect(self._on_angle_slider_changed)
        self.angle_input.editingFinished.connect(self._on_angle_input_changed)

    def _on_speed_slider_changed(self, value):
        self._update_speed_input_from_slider(value)
        self._on_slider_value_changed()

    def _on_speed_input_changed(self):
        self._update_speed_slider_from_input()
        self._on_slider_value_changed()

    def _on_angle_slider_changed(self, value):
        self._update_angle_input_from_slider(value)
        self._on_slider_value_changed()

    def _on_angle_input_changed(self):
        self._update_angle_slider_from_input()
        self._on_slider_value_changed()

    def _update_speed_input_from_slider(self, value):
        speed = value / self.SPEED_SLIDER_FACTOR
        self.speed_input.setText(f"{speed:.1f}")

    def _update_speed_slider_from_input(self):
        try:
            speed = float(self.speed_input.text())
            clamped_speed = max(-30.0, min(30.0, speed))
            self.speed_input.setText(f"{clamped_speed:.1f}")
            self.speed_slider.setValue(int(clamped_speed * self.SPEED_SLIDER_FACTOR))
        except ValueError:
            self._update_speed_input_from_slider(self.speed_slider.value())
            QMessageBox.warning(self, "입력 오류", "유효한 속도(숫자)를 입력해주세요.")

    def _update_angle_input_from_slider(self, value):
        angle = value / self.ANGLE_SLIDER_FACTOR
        self.angle_input.setText(f"{angle:.1f}")

    def _update_angle_slider_from_input(self):
        try:
            angle = float(self.angle_input.text())
            clamped_angle = max(-30.0, min(30.0, angle))
            self.angle_input.setText(f"{clamped_angle:.1f}")
            self.angle_slider.setValue(int(clamped_angle * self.ANGLE_SLIDER_FACTOR))
        except ValueError:
            self._update_angle_input_from_slider(self.angle_slider.value())
            QMessageBox.warning(self, "입력 오류", "유효한 각도(숫자)를 입력해주세요.")

    def _on_slider_value_changed(self):
        try:
            self.current_speed = float(self.speed_input.text())
            self.current_angular = float(self.angle_input.text())
            if self.bus is None:
                return
            self._send_drive_frame(self.current_speed, self.current_angular)
            if not self.drive_timer.isActive():
                self.drive_timer.start()
        except ValueError:
            pass
        except Exception as e:
            QMessageBox.critical(self, "주행 명령 오류", f"슬라이더 조작 중 오류 발생:\n{e}")
            self.drive_timer.stop()

    def connect_can_interface(self):
        """Kvaser CANlib 채널 0에 연결합니다."""
        if self.bus is not None:
            QMessageBox.warning(self, "경고", "이미 CAN 버스에 연결되어 있습니다.")
            return

        if not CANLIB_AVAILABLE or canlib is None:
            QMessageBox.critical(self, "오류", "이 시스템에서 Kvaser CANlib(libcanlib.so)을 사용할 수 없습니다.")
            return

        try:
            self.bus = canlib.openChannel(self.channel_num)
            self.bus.setBusParams(canlib.Bitrate.BITRATE_500K)
            self.bus.busOn()
            self.read_timer.start(50)
            QMessageBox.information(self, "정보", f"Kvaser CANlib 채널 {self.channel_num} (500K) 연결 성공.")
            self._on_slider_value_changed() 
        except canlib.CanError as e:
            self.bus = None
            QMessageBox.critical(self, "오류", f"Kvaser CANlib 연결 실패:\n{e}")
        except Exception as e:
            self.bus = None
            QMessageBox.critical(self, "오류", f"CAN 연결 실패:\n{e}")

    def disconnect_can_interface(self):
        """CAN 버스 연결을 해제합니다."""
        if self.bus is not None:
            self.read_timer.stop()
            self.drive_timer.stop()
            try:
                self.bus.busOff()
                self.bus.close()
            except Exception:
                pass
            self.bus = None
            QMessageBox.information(self, "정보", "CAN 버스 연결 해제됨.")
        else:
            QMessageBox.warning(self, "경고", "CAN 버스가 연결되어 있지 않습니다.")

    def _read_can_messages(self):
        """CAN 버스에서 메시지를 읽고 테이블을 업데이트합니다."""
        if self.bus is None or canlib is None:
            return
        try:
            for _ in range(100):
                try:
                    frame = self.bus.read(timeout=0)
                    if frame is None:
                        break
                    self._update_raw_table(frame)
                    self._update_parsed_table(frame)
                except canlib.CanNoMsg:
                    break
        except Exception as e:
            if self.bus is not None:
                self.read_timer.stop()
                QMessageBox.critical(self, "CAN 읽기 오류", f"메시지 읽기 중 오류 발생:\n{e}")
                self.disconnect_can_interface()

    def _send_can_frame(self):
        self._generic_send_can_frame(self.input_id, self.input_data, "Write CAN 1")

    def _send_can_frame2(self):
        self._generic_send_can_frame(self.input_id2, self.input_data2, "Write CAN 2")

    def _generic_send_can_frame(self, id_input_field, data_input_fields, error_title):
        try:
            if self.bus is None or Frame is None:
                raise Exception("CAN 버스가 연결되어 있지 않거나 CANlib을 이용할 수 없습니다.")

            can_id_text = id_input_field.text().strip()
            if not can_id_text:
                raise ValueError("CAN ID를 입력해주세요.")

            try:
                can_id = int(can_id_text, 16)
            except ValueError:
                raise ValueError("유효한 16진수 CAN ID를 입력해주세요.")

            flags = canlib.MessageFlag.EXT if can_id > 0x7FF else canlib.MessageFlag.STD

            data = []
            for i, field in enumerate(data_input_fields):
                byte_text = field.text().strip()
                if byte_text:
                    try:
                        byte_val = int(byte_text, 16)
                        if not (0 <= byte_val <= 255):
                            raise ValueError(f"데이터 바이트 {i+1}의 값이 유효한 16진수 범위(00-FF)를 벗어났습니다.")
                    except ValueError as ve:
                        raise ValueError(f"데이터 바이트 {i+1}에 유효하지 않은 16진수 입력: '{byte_text}' ({ve})")
                else:
                    byte_val = 0x00
                data.append(byte_val)

            while data and data[-1] == 0x00 and not data_input_fields[len(data) - 1].text().strip():
                data.pop()

            frame = Frame(id_=can_id, data=bytes(data), flags=flags)
            self.bus.write(frame)
        except Exception as e:
            QMessageBox.critical(self, f"전송 오류 ({error_title})", str(e))

    def _send_drive_frame(self, speed, angular):
        if self.bus is None or Frame is None:
            return

        angular = max(-30.0, min(30.0, angular))

        gear = 0x2
        if speed > 0.1:
            gear = 0x1
        elif speed < -0.1:
            gear = 0x3
            speed = abs(speed)
        else:
            gear = 0x2

        left_turn = 1 if angular > 5.0 else 0
        right_turn = 1 if angular < -5.0 else 0

        speed_val_for_504 = int(speed / 0.1)
        linear_v1 = speed_val_for_504 & 0xFF
        linear_v2 = (speed_val_for_504 >> 8) & 0xFF

        angular_val_for_502 = int((angular + 30) / 0.1)
        angular_v1 = angular_val_for_502 & 0xFF
        angular_v2 = (angular_val_for_502 >> 8) & 0xFF

        cntr = self.ad_msg_counter
        valid_bit = 0x1 if self.ad_mode_requested else 0x0
        valid_header = (cntr << 4) | valid_bit
        body_header = (cntr << 4) | (left_turn & 0x1) | ((right_turn & 0x1) << 1)

        msgs = [
            Frame(id_=0x501, data=bytes([valid_header, 0, 0, 0, 0, 0, 0, 0])),
            Frame(id_=0x503, data=bytes([valid_header, 0, 0, 0, 0, 0, 0, 0])),
            Frame(id_=0x502, data=bytes([valid_header, 0, 0, 0, angular_v1, angular_v2, 0, 0])),
            Frame(id_=0x506, data=bytes([body_header, 0, 0, 0, 0, 0, 0, 0])),
            Frame(id_=0x504, data=bytes([valid_header, 0x00, 0x01, gear, 0x32, 0, linear_v1, linear_v2]))
        ]

        try:
            for m in msgs:
                pass
                #self.bus.write(m)
        except canlib.CanError as e:
            if getattr(e, 'status', None) == getattr(canlib, 'ErrorNumber', None) and getattr(canlib.ErrorNumber, 'TXBUFOVRFL', None) == e.status:
                pass
            elif getattr(e, 'param', None) == -13 or "overflow" in str(e).lower():
                pass
            else:
                self.drive_timer.stop()
                self.sent_frames_label.setText(f"!!! 전송 실패로 타이머 정지: {e!r}")
                QMessageBox.critical(self, "CAN 전송 오류",
                                      f"프레임 전송 중 오류가 발생해 반복 전송을 멈췄습니다.\n\n{e!r}")
                return

        req_text = "Valid(요청중)" if self.ad_mode_requested else "Invalid(요청안함)"
        lines = [f"전송된 프레임 (Request_Flag={req_text}, heartbeat={cntr}):"]
        for m in msgs:
            lines.append(f"  0x{m.id:03X}: " + " ".join(f"{b:02X}" for b in m.data))
        self.sent_frames_label.setText("\n".join(lines))

        self.ad_msg_counter = (self.ad_msg_counter + 1) % 16

    def _send_repeated_drive_command(self):
        self._send_drive_frame(self.current_speed, self.current_angular)

    def _stop_vehicle(self):
        if self.bus:
            self.ad_mode_requested = False
            self.mode_request_label.setText("현재 요청 모드: Remote Control")
            self.drive_timer.stop()
            self._send_drive_frame(0.0, 0.0)
            self.speed_slider.setValue(0)
            self.angle_slider.setValue(0)
            QMessageBox.information(self, "정보", "차량 정지 명령 전송됨.")
        else:
            QMessageBox.warning(self, "경고", "CAN 버스가 연결되어 있지 않아 정지 명령을 보낼 수 없습니다.")

    def _request_ad_mode(self):
        if self.bus is None:
            QMessageBox.warning(self, "경고", "CAN 버스가 연결되어 있지 않습니다.")
            return
        self.ad_mode_requested = True
        self.mode_request_label.setText("현재 요청 모드: AD Mode")
        self._send_drive_frame(self.current_speed, self.current_angular)
        if not self.drive_timer.isActive():
            self.drive_timer.start()

    def _request_remote_control_mode(self):
        if self.bus is None:
            QMessageBox.warning(self, "경고", "CAN 버스가 연결되어 있지 않습니다.")
            return
        self.ad_mode_requested = False
        self.mode_request_label.setText("현재 요청 모드: Remote Control")
        self._send_drive_frame(self.current_speed, self.current_angular)

    def _open_ad_control_panel(self):
        if self.ad_control_panel is None:
            self.ad_control_panel = ADControlPanel(self)
        if not self.ad_control_panel.timer.isActive():
            self.ad_control_panel.timer.start(20)
        self.ad_control_panel.show()
        self.ad_control_panel.raise_()
        self.ad_control_panel.activateWindow()

    def _update_raw_table(self, message):
        can_id = hex(getattr(message, 'id', getattr(message, 'arbitration_id', 0)))
        data_hex = message.data.hex()
        dlc = str(getattr(message, 'dlc', len(message.data)))

        for row in range(self.raw_table.rowCount()):
            if self.raw_table.item(row, 0).text() == can_id:
                self.raw_table.setItem(row, 1, QTableWidgetItem(dlc))
                self.raw_table.setItem(row, 2, QTableWidgetItem(data_hex))
                return
        
        row_count = self.raw_table.rowCount()
        self.raw_table.insertRow(row_count)
        self.raw_table.setItem(row_count, 0, QTableWidgetItem(can_id))
        self.raw_table.setItem(row_count, 1, QTableWidgetItem(dlc))
        self.raw_table.setItem(row_count, 2, QTableWidgetItem(data_hex))

    def _update_parsed_table(self, message):
        can_id = getattr(message, 'id', getattr(message, 'arbitration_id', 0))
        parsed_data = self.parser.parse(can_id, message.data)
        for name, value in parsed_data.items():
            found = False
            for row in range(self.parsed_table.rowCount()):
                item = self.parsed_table.item(row, 0)
                if item and item.text() == name:
                    self.parsed_table.setItem(row, 1, QTableWidgetItem(value))
                    found = True
                    break
            if not found:
                row = self.parsed_table.rowCount()
                self.parsed_table.insertRow(row)
                self.parsed_table.setItem(row, 0, QTableWidgetItem(name))
                self.parsed_table.setItem(row, 1, QTableWidgetItem(value))

    def clear_tables(self):
        self.raw_table.setRowCount(0)
        self.parsed_table.setRowCount(0)
        QMessageBox.information(self, "정보", "모든 테이블이 초기화되었습니다.")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())