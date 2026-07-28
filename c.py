
import sys
import time
import can
import json
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem,
    QLabel, QHBoxLayout, QPushButton, QFileDialog, QMessageBox, QLineEdit
)
from PyQt6.QtCore import QTimer

CONFIG_FILE = "can_config.json"

class CANParser:
    def __init__(self):
        pass

    def parse(self, can_id, data):
        parsed = {}
        if can_id == 0x303:
            vehicle_Gear = data[0] & 0x03
            parsed['Vehicle Gear'] = ["P Gear", "D Gear", "N Gear", "R Gear"][vehicle_Gear]
            Drive_State_Mode = data[1] & 0x03
            parsed['Drive_State_Mode'] = ["Remote Control Mode", "Represents the AD Mode",
                                          "Indicates parallel Mode", "Indicates semi-autonomous"][Drive_State_Mode]
            VCU_Speed_Req = int.from_bytes(data[2:4], byteorder='little') * 0.1 - 80
            parsed['Vehicle Speed Request (km/h)'] = f"{VCU_Speed_Req:.1f}"

        elif can_id == 0x314:
            directionalAngle = int.from_bytes(data[1:3], byteorder='little')
            parsed['Direction Angle (deg)'] = f"{directionalAngle}"
            parsed['eps Control'] = "Works" if data[0] & 0x01 else "Stops"

        elif can_id == 0x304:
            speed = int.from_bytes(data[0:2], byteorder='little') * 0.1 - 80
            parsed['Vehicle Speed (km/h)'] = f"{speed:.1f}"
            parsed['Vehicle Wheel End Angle (deg)'] = f"{int.from_bytes(data[4:6], 'little') * 0.1 - 35:.1f}"
            parsed['Vehicle Break Pressure (Mps)'] = f"{int.from_bytes(data[2:4], 'little') * 0.01:.2f}"

        elif can_id == 0x301:
            parsed['Brake Light'] = "ON" if data[5] & 0x01 else "OFF"
            parsed['Head Light'] = "ON" if data[1] & 0x80 else "OFF"
            parsed['Emergency Button'] = "Pressed" if data[0] & 0x01 else "Not Pressed"
            parsed['Back Touch Switch State'] = "trigger" if data[1] & 0x20 else "Not trigger"
            parsed['Front Touch Switch State'] = "trigger" if data[1] & 0x10 else "Not trigger"

        elif can_id == 0x18F:
            parsed['EPS_Current_Angle (deg)'] = str(int.from_bytes(data[1:3], 'little', signed=True))
            parsed['EPS_ECU_Temperature (℃)'] = str(int.from_bytes(data[6:7], 'little', signed=True))

        elif can_id == 0x060:
            parsed['BUS Voltage (V)'] = f"{int.from_bytes(data[0:2], 'little') * 0.1:.2f}"
            parsed['BUS Current (A)'] = f"{int.from_bytes(data[2:4], 'little') * 0.1 - 1000:.2f}"

        elif can_id == 0x160:
            mode = (data[0] & 0x06) >> 1
            parsed['Drive Mode'] = ["Torque", "Speed", "Torque ring", "Speed loop"][mode]
            parsed['MCU_Brake_Request'] = "Hold brake" if data[0] & 0x08 else "Release"
            parsed['MCU Speed Request (RPM)'] = str(int.from_bytes(data[3:6], 'little') - 7000)
            parsed['MCU Torque Request (Nm)'] = f"{int.from_bytes(data[1:3], 'little') * 0.1 - 1000:.1f}"

        elif can_id == 0x0A0:
            parsed['BMS Battery SOH (%)'] = str(data[7])
            parsed['BMS Battery SOC (%)'] = f"{data[4] * 0.4:.2f}"
            parsed['BMS Battery Voltage (V)'] = f"{int.from_bytes(data[2:4], 'little') * 0.1:.2f}"

        return parsed

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Withus CAN Monitor")
        self.resize(1200, 800)
        self.interface_name = self.load_config()
        self.parser = CANParser()

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        self.interface_label = QLabel(f"Interface: {self.interface_name}")
        layout.addWidget(self.interface_label)

        btn_layout = QHBoxLayout()
        self.btn_connect = QPushButton("Connect CAN")
        self.btn_connect.clicked.connect(self.connect_can_interface)
        self.btn_disconnect = QPushButton("Disconnect CAN")
        self.btn_disconnect.clicked.connect(self.disconnect_can_interface)
        self.btn_clear = QPushButton("Clear Tables")
        self.btn_clear.clicked.connect(self.clear_tables)
        for btn in [self.btn_connect, self.btn_disconnect, self.btn_clear]:
            btn_layout.addWidget(btn)
        layout.addLayout(btn_layout)

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

        control_layout = QHBoxLayout()
        self.speed_input = QLineEdit()
        self.speed_label = QLabel("Speed (km/h):")
        self.speed_input.setPlaceholderText("Enter Speed (km/h)")
        
        
        self.angle_input = QLineEdit()
        self.angle_label = QLabel("Angular (deg.):")
        self.angle_input.setPlaceholderText("Enter Angle (Deg.)")
        self.btn_send_drive = QPushButton("Send Drive Command")
        self.btn_send_drive.clicked.connect(self.send_drive_command)
        self.btn_stop = QPushButton("Stop Vehicle")
        self.btn_stop.clicked.connect(self.stop_vehicle)

        control_layout.addWidget(self.speed_label)
        control_layout.addWidget(self.speed_input)
        control_layout.addWidget(self.angle_label)
        control_layout.addWidget(self.angle_input)
        control_layout.addWidget(self.btn_send_drive)
        control_layout.addWidget(self.btn_stop)
        layout.addLayout(control_layout)

        form_layout = QHBoxLayout()
        self.input_id = QLineEdit()
        self.input_id.setPlaceholderText("CAN ID (hex)")
        self.input_data = []
        form_layout.addWidget(self.input_id)
        for i in range(8):
            field = QLineEdit()
            field.setMaxLength(2)
            field.setPlaceholderText(f"{i}")
            self.input_data.append(field)
            form_layout.addWidget(field)
        self.btn_write = QPushButton("Write CAN")
        self.btn_write.clicked.connect(self.send_can_frame)
        form_layout.addWidget(self.btn_write)
        layout.addLayout(form_layout)


        form_layout2 = QHBoxLayout()
        self.input_id2 = QLineEdit()
        self.input_id2.setPlaceholderText("CAN ID (hex)")
        self.input_data2 = []
        form_layout2.addWidget(self.input_id2)
        for i in range(8):
            field = QLineEdit()
            field.setMaxLength(2)
            field.setPlaceholderText(f"{i}")
            self.input_data2.append(field)
            form_layout2.addWidget(field)
        self.btn_write2 = QPushButton("Write CAN")
        self.btn_write2.clicked.connect(self.send_can_frame2)
        form_layout2.addWidget(self.btn_write2)
        layout.addLayout(form_layout2)

        self.bus = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_can_messages)

        self.drive_timer = QTimer()
        self.drive_timer.timeout.connect(self.send_repeated_drive_command)
        self.current_speed = 0.0

    def load_config(self):
        try:
            with open(CONFIG_FILE, 'r') as f:
                return json.load(f).get("interface", "can0")
        except:
            return "can0"

    def connect_can_interface(self):
        if self.bus:
            QMessageBox.warning(self, "Warning", "Already connected.")
            return
        try:
            self.bus = can.Bus(channel=self.interface_name, interface='socketcan')
            self.timer.start(50)
            QMessageBox.information(self, "Info", "CAN connected.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Connection failed:\n{e}")

    def disconnect_can_interface(self):
        if self.bus:
            self.timer.stop()
            self.drive_timer.stop()
            self.bus.shutdown()
            self.bus = None
            QMessageBox.information(self, "Info", "CAN disconnected.")

    def read_can_messages(self):
        try:
            for _ in range(100):
                msg = self.bus.recv(timeout=0.0)
                if msg is None:
                    break
                self.update_raw_table(msg)
                self.update_parsed_table(msg)
        except Exception as e:
            print(f"Read error: {e}")

    def send_can_frame_bak(self):
        try:
            if self.bus is None:
                raise Exception("Not connected.")
            can_id = int(self.input_id.text(), 16)
            data = [int(f.text(), 16) for f in self.input_data if f.text()]
            if len(data) != 8:
                raise ValueError("DLC must be 8.")
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            self.bus.send(msg)
        except Exception as e:
            QMessageBox.critical(self, "Send Error", str(e))

    def send_can_frame(self):
        try:
            if self.bus is None:
                raise Exception("Not connected.")
            can_id_text = self.input_id.text().strip()
            if not can_id_text:
                raise ValueError("CAN ID is empty.")
            
            can_id = int(can_id_text, 16)
            is_extended = can_id > 0x7FF

            data = [int(f.text(), 16) for f in self.input_data if f.text()]
            if len(data) != 8:
                raise ValueError("DLC must be 8.")
            
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=is_extended)
            self.bus.send(msg)
        except Exception as e:
            QMessageBox.critical(self, "Send Error", str(e))

    def send_can_frame2(self):
        try:
            if self.bus is None:
                raise Exception("Not connected.")
            
            can_id = int(self.input_id2.text(), 16)
            data = [int(f.text(), 16) for f in self.input_data2 if f.text()]
            if len(data) != 8:
                raise ValueError("DLC must be 8.")
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            self.bus.send(msg)
        except Exception as e:
            QMessageBox.critical(self, "Send Error (Write2)", str(e))

    def send_drive_command(self):
        try:
            if self.bus is None:
                raise Exception("CAN not connected.")
            
            speed_text = self.speed_input.text().strip()
            if not speed_text:
                raise ValueError("Speed field is empty.")
            try:
                speed = float(speed_text)
            except ValueError:
                raise ValueError("Speed must be a valid number.")

            speed = float(speed_text)
            self.current_speed = speed
            
            #
            angular_text = self.angle_input.text().strip()
            if not angular_text:
                raise ValueError("Angular field is empty.")

            try:
                angular = float(angular_text)
            except ValueError:
                raise ValueError("Angular must be a valid number.")

            self.current_angular = angular

            self.send_drive_frame(speed, angular)
            self.drive_timer.start(500)
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    def send_drive_frame(self, speed, angular):
        gear = None
        if speed < 0: #BACKWARD Reverse Gear
            gear = 0x3
            speed = abs(speed)
        elif speed == 0 and angular == 0: # Neutral Gear
            gear = 0x2
        else:               # Forward Driving Gear 
            gear = 0x1

        indicator = None
        if angular < 0:     # 우회전 깜빡이
            indicator= 0xF2
        elif angular > 0:   # 좌회전 깜빡이
            indicator = 0xF1
        else:
            indicator = 0x0 # 직진



        speed_val = int(speed / 0.1)
        linear_v1 = speed_val & 0xFF
        linear_v2 = (speed_val >> 8) & 0xFF

        angular_velocity = angular
        angular_velocity = int((angular_velocity + 30) / 0.1)
        angular_v1 = angular_velocity & 0xFF
        angular_v2 = (angular_velocity >> 8) & 0xFF

        msgs = [
            can.Message(arbitration_id=0x501, data=[0xF1, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False),
            can.Message(arbitration_id=0x503, data=[0xF1, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False),
            can.Message(arbitration_id=0x502, data=[0xF1, 0, 0, 0, angular_v1, angular_v2, 0, 0], is_extended_id=False),
            can.Message(arbitration_id=0x506, data=[indicator, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False),
            can.Message(arbitration_id=0x504, data=[0xF1, 0x00, 0x01, gear, 0, 0, linear_v1, linear_v2], is_extended_id=False)
        ]
        for m in msgs:
            self.bus.send(m)
            time.sleep(0.01)

    def send_repeated_drive_command(self):
        self.send_drive_frame(self.current_speed, self.current_angular)

    def stop_vehicle(self):
        if self.bus:
            self.drive_timer.stop()
            self.send_drive_frame(0.0, 0.0)
            #QMessageBox.information(self, "정지", "차량 정지 명령 전송됨.")

    def update_raw_table(self, message):
        can_id = hex(message.arbitration_id)
        row_count = self.raw_table.rowCount()
        for row in range(row_count):
            if self.raw_table.item(row, 0).text() == can_id:
                self.raw_table.setItem(row, 1, QTableWidgetItem(str(message.dlc)))
                self.raw_table.setItem(row, 2, QTableWidgetItem(message.data.hex()))
                return
        self.raw_table.insertRow(row_count)
        self.raw_table.setItem(row_count, 0, QTableWidgetItem(can_id))
        self.raw_table.setItem(row_count, 1, QTableWidgetItem(str(message.dlc)))
        self.raw_table.setItem(row_count, 2, QTableWidgetItem(message.data.hex()))

    def update_parsed_table(self, message):
        parsed = self.parser.parse(message.arbitration_id, message.data)
        for name, value in parsed.items():
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

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
