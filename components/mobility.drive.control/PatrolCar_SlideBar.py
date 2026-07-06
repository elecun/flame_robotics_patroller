import sys
import time
import can
import json
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem,
    QLabel, QHBoxLayout, QPushButton, QMessageBox, QLineEdit, QSlider
)
from PyQt6.QtCore import QTimer, Qt

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

        # 0x314: 방향각, EPS 제어 상태
        elif can_id == 0x314:
            if len(data) >= 3: # 데이터 길이 확인
                directional_angle = int.from_bytes(data[1:3], byteorder='little')
                parsed['Direction Angle (deg)'] = f"{directional_angle}"
                parsed['eps Control'] = "Works" if data[0] & 0x01 else "Stops"

        # 0x304: 차량 속도, 휠 엔드 각도, 브레이크 압력
        elif can_id == 0x304:
            if len(data) >= 6: # 데이터 길이 확인
                speed = int.from_bytes(data[0:2], byteorder='little') * 0.1 - 80
                parsed['Vehicle Speed (km/h)'] = f"{speed:.1f}"
                parsed['Vehicle Wheel End Angle (deg)'] = f"{int.from_bytes(data[4:6], 'little') * 0.1 - 35:.1f}"
                parsed['Vehicle Break Pressure (Mps)'] = f"{int.from_bytes(data[2:4], 'little') * 0.01:.2f}"

        # 0x301: 라이트, 스위치 상태
        elif can_id == 0x301:
            if len(data) >= 6: # 데이터 길이 확인
                parsed['Brake Light'] = "ON" if data[5] & 0x01 else "OFF"
                parsed['Head Light'] = "ON" if data[1] & 0x80 else "OFF"
                parsed['Emergency Button'] = "Pressed" if data[0] & 0x01 else "Not Pressed"
                parsed['Back Touch Switch State'] = "trigger" if data[1] & 0x20 else "Not trigger"
                parsed['Front Touch Switch State'] = "trigger" if data[1] & 0x10 else "Not trigger"

        # 0x18F: EPS 현재 각도, ECU 온도
        elif can_id == 0x18F:
            if len(data) >= 7: # 데이터 길이 확인
                parsed['EPS_Current_Angle (deg)'] = str(int.from_bytes(data[1:3], 'little', signed=True))
                parsed['EPS_ECU_Temperature (℃)'] = str(int.from_bytes(data[6:7], 'little', signed=True))

        # 0x060: 버스 전압, 버스 전류
        elif can_id == 0x060:
            if len(data) >= 4: # 데이터 길이 확인
                parsed['BUS Voltage (V)'] = f"{int.from_bytes(data[0:2], 'little') * 0.1:.2f}"
                parsed['BUS Current (A)'] = f"{int.from_bytes(data[2:4], 'little') * 0.1 - 1000:.2f}"

        # 0x160: 드라이브 모드, MCU 브레이크 요청, MCU 속도/토크 요청
        elif can_id == 0x160:
            if len(data) >= 6: # 데이터 길이 확인
                mode = (data[0] & 0x06) >> 1
                parsed['Drive Mode'] = ["Torque", "Speed", "Torque ring", "Speed loop"][mode]
                parsed['MCU_Brake_Request'] = "Hold brake" if data[0] & 0x08 else "Release"
                parsed['MCU Speed Request (RPM)'] = str(int.from_bytes(data[3:6], 'little') - 7000)
                parsed['MCU Torque Request (Nm)'] = f"{int.from_bytes(data[1:3], 'little') * 0.1 - 1000:.1f}"

        # 0x0A0: BMS 배터리 SOH, SOC, 전압
        elif can_id == 0x0A0:
            if len(data) >= 8: # 데이터 길이 확인
                parsed['BMS Battery SOH (%)'] = str(data[7])
                parsed['BMS Battery SOC (%)'] = f"{data[4] * 0.4:.2f}"
                parsed['BMS Battery Voltage (V)'] = f"{int.from_bytes(data[2:4], 'little') * 0.1:.2f}"

        return parsed

class MainWindow(QMainWindow):
    """
    CAN 통신 모니터링 및 제어를 위한 메인 GUI 창 클래스.
    """
    # 슬라이더 스케일링을 위한 상수 정의
    SPEED_SLIDER_FACTOR = 10.0 # 10 = 0.1km/h 단위 (슬라이더 값 10 -> 1.0 km/h)
    ANGLE_SLIDER_FACTOR = 10.0  # 10 = 0.1도 단위 (슬라이더 값 10 -> 1.0 deg)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Withus CAN Monitor")
        self.resize(1200, 800)
        self.interface_name = self._load_config()
        self.parser = CANParser()

        self._setup_ui() # UI 설정 메서드 호출
        self._connect_signals_slots() # 시그널-슬롯 연결 메서드 호출

        self.bus = None
        self.read_timer = QTimer()
        self.read_timer.timeout.connect(self._read_can_messages)

        self.drive_timer = QTimer()
        self.drive_timer.timeout.connect(self._send_repeated_drive_command)
        # drive_timer의 주기를 더 짧게 변경 (예: 50ms 또는 100ms)
        self.drive_timer.setInterval(100) # 100ms마다 반복 전송

        self.current_speed = 0.0
        self.current_angular = 0.0

        # 초기값 설정 (슬라이더와 입력 필드 연동)
        self._update_speed_input_from_slider(self.speed_slider.value())
        self._update_angle_input_from_slider(self.angle_slider.value())


    def _setup_ui(self):
        """UI 요소를 설정합니다."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # 인터페이스 정보 및 연결/해제/초기화 버튼
        self.interface_label = QLabel(f"Interface: {self.interface_name}")
        layout.addWidget(self.interface_label)

        btn_layout = QHBoxLayout()
        self.btn_connect = QPushButton("Connect CAN")
        self.btn_disconnect = QPushButton("Disconnect CAN")
        self.btn_clear = QPushButton("Clear Tables")
        for btn in [self.btn_connect, self.btn_disconnect, self.btn_clear]:
            btn_layout.addWidget(btn)
        layout.addLayout(btn_layout)

        # Raw 데이터 및 파싱된 데이터 테이블
        table_layout = QHBoxLayout()
        self.raw_table = QTableWidget(0, 3)
        self.raw_table.setHorizontalHeaderLabels(["CAN ID", "DLC", "Data"])
        self.raw_table.setColumnWidth(0, 100)
        self.raw_table.setColumnWidth(1, 50)
        self.raw_table.setColumnWidth(2, 200)
        table_layout.addWidget(self.raw_table)

        self.parsed_table = QTableWidget(0, 2)
        self.parsed_table.setHorizontalHeaderLabels(["Name", "Value"])
        self.parsed_table.setColumnWidth(0, 300)
        self.parsed_table.setColumnWidth(1, 200)
        table_layout.addWidget(self.parsed_table)
        layout.addLayout(table_layout)

        # 차량 제어 섹션 (속도, 각도, 전송, 정지) - 슬라이더 추가
        control_layout = QVBoxLayout() # 수직 레이아웃으로 변경하여 각 제어 그룹을 수직으로 쌓음

        # 속도 제어 그룹
        speed_control_group = QHBoxLayout()
        self.speed_label = QLabel("Speed (km/h):")
        self.speed_input = QLineEdit("0.0") # 초기값 설정
        self.speed_input.setPlaceholderText("Enter Speed (km/h)")
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(int(-30 * self.SPEED_SLIDER_FACTOR), int(30 * self.SPEED_SLIDER_FACTOR)) # -30km/h ~ 30km/h (0.1 단위)
        self.speed_slider.setValue(0) # 초기값 0
        self.speed_slider.setSingleStep(1)
        self.speed_slider.setPageStep(10)
        self.speed_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.speed_slider.setTickInterval(50) # 5km/h 간격으로 눈금 표시

        speed_control_group.addWidget(self.speed_label)
        speed_control_group.addWidget(self.speed_input)
        speed_control_group.addWidget(self.speed_slider)
        control_layout.addLayout(speed_control_group)

        # 각도 제어 그룹
        angle_control_group = QHBoxLayout()
        self.angle_label = QLabel("Angular (deg.):")
        self.angle_input = QLineEdit("0.0") # 초기값 설정
        self.angle_input.setPlaceholderText("Enter Angle (Deg.)")
        self.angle_slider = QSlider(Qt.Orientation.Horizontal)
        # --- MODIFICATION START ---
        self.angle_slider.setRange(int(-30 * self.ANGLE_SLIDER_FACTOR), int(30 * self.ANGLE_SLIDER_FACTOR)) # -30 deg ~ 30 deg (0.1도 단위)
        # --- MODIFICATION END ---
        self.angle_slider.setValue(0) # 초기값 0
        self.angle_slider.setSingleStep(1)
        self.angle_slider.setPageStep(10)
        self.angle_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.angle_slider.setTickInterval(50) # 5도 간격으로 눈금 표시

        angle_control_group.addWidget(self.angle_label)
        angle_control_group.addWidget(self.angle_input)
        angle_control_group.addWidget(self.angle_slider)
        control_layout.addLayout(angle_control_group)

        # 전송 및 정지 버튼
        action_buttons_layout = QHBoxLayout()
        # self.btn_send_drive = QPushButton("Send Drive Command") # 삭제: 슬라이더에 직접 연결
        self.btn_stop = QPushButton("Stop Vehicle")
        # action_buttons_layout.addWidget(self.btn_send_drive) # 삭제: 슬라이더에 직접 연결
        action_buttons_layout.addWidget(self.btn_stop)
        control_layout.addLayout(action_buttons_layout)

        layout.addLayout(control_layout) # 메인 레이아웃에 컨트롤 레이아웃 추가

        # 수동 CAN 메시지 전송 섹션 1
        form_layout = QHBoxLayout()
        self.input_id = QLineEdit()
        self.input_id.setPlaceholderText("CAN ID (hex)")
        self.input_data = []
        form_layout.addWidget(self.input_id)
        for i in range(8):
            field = QLineEdit()
            field.setMaxLength(2)
            field.setPlaceholderText(f"Byte {i:X}")
            self.input_data.append(field)
            form_layout.addWidget(field)
        self.btn_write = QPushButton("Write CAN 1")
        layout.addLayout(form_layout)

        # 수동 CAN 메시지 전송 섹션 2
        form_layout2 = QHBoxLayout()
        self.input_id2 = QLineEdit()
        self.input_id2.setPlaceholderText("CAN ID (hex)")
        self.input_data2 = []
        form_layout2.addWidget(self.input_id2)
        for i in range(8):
            field = QLineEdit()
            field.setMaxLength(2)
            field.setPlaceholderText(f"Byte {i:X}")
            self.input_data2.append(field)
            form_layout2.addWidget(field)
        self.btn_write2 = QPushButton("Write CAN 2")
        layout.addLayout(form_layout2)

    def _connect_signals_slots(self):
        """UI 요소의 시그널과 슬롯을 연결합니다."""
        self.btn_connect.clicked.connect(self.connect_can_interface)
        self.btn_disconnect.clicked.connect(self.disconnect_can_interface)
        self.btn_clear.clicked.connect(self.clear_tables)
        # self.btn_send_drive.clicked.connect(self._send_drive_command) # 삭제
        self.btn_stop.clicked.connect(self._stop_vehicle)
        self.btn_write.clicked.connect(self._send_can_frame)
        self.btn_write2.clicked.connect(self._send_can_frame2)

        # 슬라이더 -> QLineEdit 연동
        self.speed_slider.valueChanged.connect(self._update_speed_input_from_slider)
        self.angle_slider.valueChanged.connect(self._update_angle_input_from_slider)

        # QLineEdit -> 슬라이더 연동 (무한 루프 방지 로직 포함)
        self.speed_input.editingFinished.connect(self._update_speed_slider_from_input)
        self.angle_input.editingFinished.connect(self._update_angle_slider_from_input)

        # 슬라이더 값 변경 시 즉시 주행 명령 전송
        self.speed_slider.valueChanged.connect(self._on_slider_value_changed)
        self.angle_slider.valueChanged.connect(self._on_slider_value_changed)

    def _update_speed_input_from_slider(self, value):
        """속도 슬라이더 값 변경 시 텍스트 입력 필드를 업데이트합니다."""
        self.speed_input.blockSignals(True) # 시그널 일시 중지
        self.speed_input.setText(f"{value / self.SPEED_SLIDER_FACTOR:.1f}")
        self.speed_input.blockSignals(False) # 시그널 재개

    def _update_angle_input_from_slider(self, value):
        """각도 슬라이더 값 변경 시 텍스트 입력 필드를 업데이트합니다."""
        self.angle_input.blockSignals(True) # 시그널 일시 중지
        self.angle_input.setText(f"{value / self.ANGLE_SLIDER_FACTOR:.1f}")
        self.angle_input.blockSignals(False) # 시그널 재개

    def _update_speed_slider_from_input(self):
        """속도 텍스트 입력 필드 변경 완료 시 슬라이더를 업데이트합니다."""
        try:
            speed = float(self.speed_input.text())
            slider_value = int(speed * self.SPEED_SLIDER_FACTOR)
            # 슬라이더 범위 내인지 확인 후 설정
            if self.speed_slider.minimum() <= slider_value <= self.speed_slider.maximum():
                self.speed_slider.blockSignals(True) # 시그널 일시 중지
                self.speed_slider.setValue(slider_value)
                self.speed_slider.blockSignals(False) # 시그널 재개
                self._on_slider_value_changed() # 입력 후 엔터 쳤을 때도 바로 전송
            else:
                QMessageBox.warning(self, "입력 오류", f"속도 값이 허용 범위(-{self.speed_slider.maximum()/self.SPEED_SLIDER_FACTOR:.1f} ~ {self.speed_slider.maximum()/self.SPEED_SLIDER_FACTOR:.1f} km/h)를 벗어났습니다.")
                self.speed_input.setText(f"{self.speed_slider.value() / self.SPEED_SLIDER_FACTOR:.1f}") # 원래 슬라이더 값으로 되돌림
        except ValueError:
            QMessageBox.critical(self, "입력 오류", "유효한 숫자를 입력해주세요.")
            self.speed_input.setText(f"{self.speed_slider.value() / self.SPEED_SLIDER_FACTOR:.1f}") # 유효하지 않으면 원래 슬라이더 값으로 되돌림

    def _update_angle_slider_from_input(self):
        """각도 텍스트 입력 필드 변경 완료 시 슬라이더를 업데이트합니다."""
        try:
            angle = float(self.angle_input.text())
            slider_value = int(angle * self.ANGLE_SLIDER_FACTOR)
            # 슬라이더 범위 내인지 확인 후 설정
            # --- MODIFICATION START ---
            if self.angle_slider.minimum() <= slider_value <= self.angle_slider.maximum():
                self.angle_slider.blockSignals(True) # 시그널 일시 중지
                self.angle_slider.setValue(slider_value)
                self.angle_slider.blockSignals(False) # 시그널 재개
                self._on_slider_value_changed() # 입력 후 엔터 쳤을 때도 바로 전송
            else:
                QMessageBox.warning(self, "입력 오류", f"각도 값이 허용 범위(-{self.angle_slider.maximum()/self.ANGLE_SLIDER_FACTOR:.1f} ~ {self.angle_slider.maximum()/self.ANGLE_SLIDER_FACTOR:.1f} deg)를 벗어났습니다.")
                self.angle_input.setText(f"{self.angle_slider.value() / self.ANGLE_SLIDER_FACTOR:.1f}") # 원래 슬라이더 값으로 되돌림
            # --- MODIFICATION END ---
        except ValueError:
            QMessageBox.critical(self, "입력 오류", "유효한 숫자를 입력해주세요.")
            self.angle_input.setText(f"{self.angle_slider.value() / self.ANGLE_SLIDER_FACTOR:.1f}") # 유효하지 않으면 원래 슬라이더 값으로 되돌림

    def _on_slider_value_changed(self):
        """
        슬라이더 값이 변경될 때 호출됩니다.
        변경된 속도와 각도 값을 기반으로 주행 명령을 보냅니다.
        """
        try:
            # CAN 버스가 연결되어 있지 않으면 동작하지 않음
            if self.bus is None:
                # QMessageBox.warning(self, "경고", "CAN 버스가 연결되어 있지 않습니다.") # 너무 자주 뜰 수 있으므로 주석 처리
                self.drive_timer.stop() # 버스 연결 안 되면 타이머 중지
                return
            
            speed = float(self.speed_input.text())
            angular = float(self.angle_input.text())
            
            self.current_speed = speed
            self.current_angular = angular

            # 슬라이더 조작 시 즉시 CAN 메시지 전송
            self._send_drive_frame(speed, angular)

            # 반복 전송 타이머 시작 (이미 실행 중이면 유지)
            if not self.drive_timer.isActive():
                self.drive_timer.start() # self.drive_timer.setInterval(100)으로 설정된 주기로 시작
            
        except ValueError:
            # 유효하지 않은 숫자 입력 시 발생하는 오류는 이미 _update_X_slider_from_input에서 처리됨
            pass
        except Exception as e:
            QMessageBox.critical(self, "주행 명령 오류", f"슬라이더 조작 중 오류 발생:\n{e}")
            self.drive_timer.stop() # 오류 발생 시 타이머 중지

    def _load_config(self):
        """설정 파일에서 CAN 인터페이스 이름을 로드합니다."""
        try:
            with open(CONFIG_FILE, 'r') as f:
                return json.load(f).get("interface", "can0")
        except (FileNotFoundError, json.JSONDecodeError):
            return "can0"

    def connect_can_interface(self):
        """CAN 버스에 연결합니다."""
        if self.bus:
            QMessageBox.warning(self, "경고", "이미 CAN 버스에 연결되어 있습니다.")
            return
        try:
            self.bus = can.Bus(channel=self.interface_name, interface='socketcan')
            self.read_timer.start(50) # 50ms마다 메시지 읽기 시도
            QMessageBox.information(self, "정보", f"CAN 버스 '{self.interface_name}' 연결 성공.")
            # 연결 성공 시 현재 슬라이더 값으로 즉시 전송 시작
            self._on_slider_value_changed() 
        except Exception as e:
            QMessageBox.critical(self, "오류", f"CAN 연결 실패:\n{e}")

    def disconnect_can_interface(self):
        """CAN 버스 연결을 해제합니다."""
        if self.bus:
            self.read_timer.stop()
            self.drive_timer.stop()
            self.bus.shutdown()
            self.bus = None
            QMessageBox.information(self, "정보", "CAN 버스 연결 해제됨.")
        else:
            QMessageBox.warning(self, "경고", "CAN 버스가 연결되어 있지 않습니다.")

    def _read_can_messages(self):
        """CAN 버스에서 메시지를 읽고 테이블을 업데이트합니다."""
        try:
            # 한 번에 여러 메시지를 처리하여 효율성을 높임
            for _ in range(100): # 최대 100개의 메시지 처리
                msg = self.bus.recv(timeout=0.0) # 논블로킹으로 메시지 수신
                if msg is None:
                    break # 더 이상 메시지가 없으면 종료
                self._update_raw_table(msg)
                self._update_parsed_table(msg)
        except Exception as e:
            # 읽기 중 오류 발생 시 타이머 중지 및 메시지 표시
            if self.bus: # 버스가 아직 연결 상태라면
                self.read_timer.stop()
                QMessageBox.critical(self, "CAN 읽기 오류", f"메시지 읽기 중 오류 발생:\n{e}")
                self.disconnect_can_interface() # 오류 발생 시 자동 연결 해제

    def _send_can_frame(self):
        """사용자 입력에 따라 CAN 프레임을 전송합니다."""
        self._generic_send_can_frame(self.input_id, self.input_data, "Write CAN 1")

    def _send_can_frame2(self):
        """두 번째 사용자 입력에 따라 CAN 프레임을 전송합니다."""
        self._generic_send_can_frame(self.input_id2, self.input_data2, "Write CAN 2")

    def _generic_send_can_frame(self, id_input_field, data_input_fields, error_title):
        """
        CAN 프레임 전송 로직의 공통 부분을 처리합니다.
        id_input_field: QLineEdit (CAN ID 입력)
        data_input_fields: list of QLineEdit (데이터 바이트 입력)
        error_title: 오류 메시지 박스 제목
        """
        try:
            if self.bus is None:
                raise Exception("CAN 버스가 연결되어 있지 않습니다.")

            can_id_text = id_input_field.text().strip()
            if not can_id_text:
                raise ValueError("CAN ID를 입력해주세요.")

            try:
                can_id = int(can_id_text, 16)
            except ValueError:
                raise ValueError("유효한 16진수 CAN ID를 입력해주세요.")

            is_extended = can_id > 0x7FF # 11비트 ID (Standard) 초과 시 확장 ID (Extended)

            data = []
            for i, field in enumerate(data_input_fields):
                byte_text = field.text().strip()
                if byte_text: # 입력된 값이 있는 필드만 처리
                    try:
                        byte_val = int(byte_text, 16)
                        if not (0 <= byte_val <= 255):
                            raise ValueError(f"데이터 바이트 {i+1}의 값이 유효한 16진수 범위(00-FF)를 벗어났습니다.")
                        data.append(byte_val)
                    except ValueError as ve:
                        raise ValueError(f"데이터 바이트 {i+1}에 유효하지 않은 16진수 입력: '{byte_text}' ({ve})")
            
            if len(data) > 8:
                raise ValueError("DLC(Data Length Code)는 8바이트를 초과할 수 없습니다.")
            
            # DLC는 데이터 배열의 실제 길이에 따라 자동으로 설정됩니다.
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=is_extended)
            self.bus.send(msg)
        except Exception as e:
            QMessageBox.critical(self, f"전송 오류 ({error_title})", str(e))

    def _send_drive_frame(self, speed, angular):
        """
        실제 CAN 드라이브 프레임을 구성하여 전송합니다.
        """
        if self.bus is None:
            return # CAN 버스가 연결되어 있지 않으면 전송하지 않음

        # 각도 범위 제한 (-30 ~ 30)
        angular = max(-30.0, min(30.0, angular))

        # 기어 설정 (0:P, 1:D, 2:N, 3:R)
        gear = 0x2 # 기본 N (Neutral)
        if speed > 0.1: # 전진 (정지 임계값 추가)
            gear = 0x1 # D Gear
        elif speed < -0.1: # 후진 (정지 임계값 추가)
            gear = 0x3 # R Gear
            speed = abs(speed) # 속도 값은 양수로 변환
        else: # 속도가 0에 가까우면 중립
            gear = 0x2 # N Gear

        # 방향 지시등 설정 (0: 없음, 0xF1: 좌, 0xF2: 우)
        indicator = 0x00 # 기본 (없음)
        # --- MODIFICATION START ---
        if angular < -5.0: # 우회전 (임의의 임계값, 5도 기준)
            indicator = 0xF2
        elif angular > 5.0: # 좌회전 (임의의 임계값, 5도 기준)
            indicator = 0xF1
        # --- MODIFICATION END ---

        # VCU_Speed_Req (CAN ID 0x303)에 사용되는 속도 값 변환
        # (VCU_Speed_Req = (value * 0.1) - 80 역산 -> value = (VCU_Speed_Req + 80) / 0.1)
        # 이 변환은 CANParser의 VCU_Speed_Req 계산과 역산 관계를 가집니다.
        # 여기서는 0x504 메시지 스펙에 맞춰 속도(km/h)를 직접 변환하는 것으로 가정합니다.
        speed_val_for_504 = int(speed / 0.1) # 0.1 km/h 단위
        linear_v1 = speed_val_for_504 & 0xFF
        linear_v2 = (speed_val_for_504 >> 8) & 0xFF

        # 0x502 메시지의 각도 값 변환 (현재 로직: (angular + 30) / 0.1)
        # 이 변환은 차량 제어 스펙에 따라 다를 수 있으므로 정확한 확인이 필수입니다.
        # angular 값이 -30에서 30일 때, (angular + 30)은 0에서 60
        # 이를 0.1 단위로 변환하면 0에서 600이 됩니다. (0.1 deg/LSB 기준)
        angular_val_for_502 = int((angular + 30) / 0.1) # 특정 범위 매핑 (예: -30도 ~ 30도 -> 0 ~ 600)
        angular_v1 = angular_val_for_502 & 0xFF
        angular_v2 = (angular_val_for_502 >> 8) & 0xFF

        msgs = [
            # 0x501: 더미/안전 메시지로 추정 (스펙 확인 필요)
            can.Message(arbitration_id=0x501, data=[0xF1, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False),
            # 0x503: 더미/안전 메시지로 추정 (스펙 확인 필요)
            can.Message(arbitration_id=0x503, data=[0xF1, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False),
            # 0x502: 조향 각도 제어 (angular_v1, angular_v2)
            can.Message(arbitration_id=0x502, data=[0xF1, 0, 0, 0, angular_v1, angular_v2, 0, 0], is_extended_id=False),
            # 0x506: 방향 지시등 제어
            can.Message(arbitration_id=0x506, data=[indicator, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False),
            # 0x504: 주행 명령 (기어, 속도)
            can.Message(arbitration_id=0x504, data=[0xF1, 0x00, 0x01, gear, 0, 0, linear_v1, linear_v2], is_extended_id=False)
        ]
        
        for m in msgs:
            self.bus.send(m)
            # time.sleep(0.01) # 이 지연이 UI 반응성을 저하시킬 수 있으므로 제거하거나 매우 짧게 조정
                               # 대신 QTimer 주기를 짧게 가져가는 것이 더 효율적

    def _send_repeated_drive_command(self):
        """QTimer에 의해 반복적으로 호출되어 현재 속도와 각도로 주행 명령을 전송합니다."""
        self._send_drive_frame(self.current_speed, self.current_angular)

    def _stop_vehicle(self):
        """차량을 정지시키는 명령을 전송하고 반복 전송을 중지합니다."""
        if self.bus:
            self.drive_timer.stop() # 반복 전송 타이머 중지
            self._send_drive_frame(0.0, 0.0) # 속도 0, 각도 0으로 정지 명령 전송
            # 슬라이더와 입력 필드를 0으로 초기화
            self.speed_slider.setValue(0)
            self.angle_slider.setValue(0)
            QMessageBox.information(self, "정보", "차량 정지 명령 전송됨.")
        else:
            QMessageBox.warning(self, "경고", "CAN 버스가 연결되어 있지 않아 정지 명령을 보낼 수 없습니다.")

    def _update_raw_table(self, message):
        """Raw CAN 메시지 테이블을 업데이트합니다."""
        can_id = hex(message.arbitration_id)
        data_hex = message.data.hex()
        dlc = str(message.dlc)

        # 기존 행을 찾아서 업데이트
        for row in range(self.raw_table.rowCount()):
            if self.raw_table.item(row, 0).text() == can_id:
                self.raw_table.setItem(row, 1, QTableWidgetItem(dlc))
                self.raw_table.setItem(row, 2, QTableWidgetItem(data_hex))
                return
        
        # 새 행 추가
        row_count = self.raw_table.rowCount()
        self.raw_table.insertRow(row_count)
        self.raw_table.setItem(row_count, 0, QTableWidgetItem(can_id))
        self.raw_table.setItem(row_count, 1, QTableWidgetItem(dlc))
        self.raw_table.setItem(row_count, 2, QTableWidgetItem(data_hex))

    def _update_parsed_table(self, message):
        """파싱된 CAN 메시지 테이블을 업데이트합니다."""
        parsed_data = self.parser.parse(message.arbitration_id, message.data)
        for name, value in parsed_data.items():
            found = False
            # 기존 행을 찾아서 업데이트
            for row in range(self.parsed_table.rowCount()):
                item = self.parsed_table.item(row, 0)
                if item and item.text() == name:
                    self.parsed_table.setItem(row, 1, QTableWidgetItem(value))
                    found = True
                    break
            # 새 행 추가
            if not found:
                row = self.parsed_table.rowCount()
                self.parsed_table.insertRow(row)
                self.parsed_table.setItem(row, 0, QTableWidgetItem(name))
                self.parsed_table.setItem(row, 1, QTableWidgetItem(value))

    def clear_tables(self):
        """모든 테이블의 내용을 지웁니다."""
        self.raw_table.setRowCount(0)
        self.parsed_table.setRowCount(0)
        QMessageBox.information(self, "정보", "모든 테이블이 초기화되었습니다.")

# --- 애플리케이션 실행 ---
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())