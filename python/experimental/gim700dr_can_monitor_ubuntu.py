
import sys
import time
import can
import json
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem,
    QLabel, QHBoxLayout, QPushButton, QMessageBox
)
from PyQt6.QtCore import QTimer

CONFIG_FILE = "can_config.json"

class GIM700DRParser:
    def __init__(self):
        self.node_id = 1
        self.pdo1_id = 0x180 + self.node_id  # Default TPDO1 ID
        

    def parse(self, can_id, data):
        parsed = {}
        if can_id == self.pdo1_id:
            if len(data) >= 6:
                temp = int.from_bytes(data[0:2], byteorder='little', signed=True)
                slope_z = int.from_bytes(data[2:4], byteorder='little', signed=True)
                slope_y = int.from_bytes(data[4:6], byteorder='little', signed=True)
                resolution = 0.1  # default from object 6000h = 0x64 = 0.1 deg/LSB
                parsed['Temperature (°C)'] = f"{temp}"
                parsed['Slope Z (°)'] = f"{slope_z * resolution:.2f}"
                parsed['Slope Y (°)'] = f"{slope_y * resolution:.2f}"
        return parsed

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GIM700DR CAN Monitor (Ubuntu)")
        self.resize(800, 600)
        self.interface_name = self.load_config()
        self.parser = GIM700DRParser()

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        self.interface_label = QLabel(f"Interface: {self.interface_name}")
        layout.addWidget(self.interface_label)

        button_layout = QHBoxLayout()
        self.btn_connect = QPushButton("Connect CAN")
        self.btn_connect.clicked.connect(self.connect_can)
        self.btn_disconnect = QPushButton("Disconnect CAN")
        self.btn_disconnect.clicked.connect(self.disconnect_can)
        button_layout.addWidget(self.btn_connect)
        button_layout.addWidget(self.btn_disconnect)
        layout.addLayout(button_layout)

        self.raw_table = QTableWidget(0, 3)
        self.raw_table.setHorizontalHeaderLabels(["CAN ID", "DLC", "Data"])
        layout.addWidget(self.raw_table)

        self.parsed_table = QTableWidget(0, 2)
        self.parsed_table.setHorizontalHeaderLabels(["Field", "Value"])
        layout.addWidget(self.parsed_table)

        self.bus = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_can)

    
    def send_nmt_start_remote_node(self):
        try:
            nmt_msg = can.Message(
                arbitration_id=0x000,          # NMT 명령은 ID 0x000
                data=[0x01, self.parser.node_id],  # 0x01 = Start Remote Node
                is_extended_id=False
            )
            self.bus.send(nmt_msg)
            print(f"✅ Sent NMT Start Remote Node to node {self.parser.node_id}")
        except Exception as e:
            print(f"❌ Failed to send NMT Start command: {e}")
    
    def load_config(self):
        try:
            with open(CONFIG_FILE, 'r') as f:
                return json.load(f).get("interface", "can0")
        except:
            return "can0"

    def connect_can(self):
        if self.bus:
            QMessageBox.warning(self, "Warning", "Already connected.")
            return
        try:
            self.bus = can.Bus(channel=self.interface_name, interface='socketcan')
            time.sleep(0.1)  # 약간의 지연 후 NMT 전송
            self.send_nmt_start_remote_node()  # <-- 추가된 부분
            self.timer.start(50)
            QMessageBox.information(self, "Connected", "CAN interface connected.")
        except Exception as e:
            QMessageBox.critical(self, "Connection Error", str(e))

    def disconnect_can(self):
        if self.bus:
            self.timer.stop()
            self.bus.shutdown()
            self.bus = None
            QMessageBox.information(self, "Disconnected", "CAN interface disconnected.")

    def read_can(self):
        try:
            for _ in range(10):
                msg = self.bus.recv(timeout=0.01)
                if msg is None:
                    break
                self.update_raw_table(msg)
                self.update_parsed_table(msg)
        except Exception as e:
            print("CAN read error:", e)

    def update_raw_table(self, msg):
        can_id = hex(msg.arbitration_id)
        row = self.raw_table.rowCount()
        self.raw_table.insertRow(row)
        self.raw_table.setItem(row, 0, QTableWidgetItem(can_id))
        self.raw_table.setItem(row, 1, QTableWidgetItem(str(msg.dlc)))
        self.raw_table.setItem(row, 2, QTableWidgetItem(msg.data.hex()))

    def update_parsed_table(self, msg):
        parsed = self.parser.parse(msg.arbitration_id, msg.data)
        for key, value in parsed.items():
            found = False
            for row in range(self.parsed_table.rowCount()):
                if self.parsed_table.item(row, 0).text() == key:
                    self.parsed_table.setItem(row, 1, QTableWidgetItem(value))
                    found = True
                    break
            if not found:
                row = self.parsed_table.rowCount()
                self.parsed_table.insertRow(row)
                self.parsed_table.setItem(row, 0, QTableWidgetItem(key))
                self.parsed_table.setItem(row, 1, QTableWidgetItem(value))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
