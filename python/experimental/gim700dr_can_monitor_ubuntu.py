"""
GIM700DR CAN Monitor using Kvaser CANlib (CLI/GUI dual support)
Monitors Baumer GIM700DR CANopen inclination sensor using Kvaser CANlib Python library (canlib).
Supports channel parameter (default: channel 1, 500k bitrate).
"""

import sys
import time
import json
import os

# Kvaser CANlib driver
try:
    from canlib import canlib, Frame
    HAS_CANLIB = True
except ImportError:
    HAS_CANLIB = False

# Fallback python-can library (kvaser/socketcan)
try:
    import can
    HAS_PYTHON_CAN = True
except ImportError:
    HAS_PYTHON_CAN = False

if not HAS_CANLIB and not HAS_PYTHON_CAN:
    print("❌ No CAN driver library available. Install via: pip install kvaser-canlib or pip install python-can")
    sys.exit(1)

# PyQt fallback / optional GUI detection
HAS_PYQT = False
try:
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem,
        QLabel, QHBoxLayout, QPushButton, QMessageBox
    )
    from PyQt6.QtCore import QTimer
    HAS_PYQT = True
except ImportError:
    HAS_PYQT = False

CONFIG_FILE = "can_config.json"


class GIM700DRParser:
    def __init__(self, node_id=1):
        self.node_id = node_id
        self.pdo1_id = 0x180 + self.node_id  # TPDO1 ID (0x181 for Node 1)

    def parse(self, can_id, data_bytes):
        parsed = {}
        if can_id == self.pdo1_id and len(data_bytes) >= 6:
            temp = int.from_bytes(data_bytes[0:2], byteorder='little', signed=True)
            slope_z = int.from_bytes(data_bytes[2:4], byteorder='little', signed=True)
            slope_y = int.from_bytes(data_bytes[4:6], byteorder='little', signed=True)
            resolution = 0.1  # 0.1 deg/LSB
            parsed['Temperature (°C)'] = f"{temp}"
            parsed['Slope Z (°)'] = f"{slope_z * resolution:.2f}"
            parsed['Slope Y (°)'] = f"{slope_y * resolution:.2f}"
        return parsed


def load_config_channel():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r') as f:
                cfg = json.load(f)
                return int(cfg.get("can_channel", cfg.get("channel", 1)))
        except Exception:
            pass
    return 1  # Default to CAN Channel 1 for Baumer Incline Sensor


def run_canlib_cli_monitor():
    channel_num = load_config_channel()
    parser = GIM700DRParser(node_id=1)
    print(f"🚀 Starting GIM700DR CAN Monitor (Kvaser CANlib Mode) on Channel {channel_num} (500k)...")

    ch = None
    if HAS_CANLIB:
        try:
            ch = canlib.openChannel(channel_num, flags=canlib.Open.ACCEPT_VIRTUAL)
            ch.setBusParams(canlib.Bitrate.BITRATE_500K)
            ch.busOn()
            print(f"✅ Connected to Kvaser CANlib Channel {channel_num}")
        except canlib.CanError:
            try:
                ch = canlib.openChannel(channel_num)
                ch.setBusParams(canlib.Bitrate.BITRATE_500K)
                ch.busOn()
                print(f"✅ Connected to Kvaser CANlib Channel {channel_num}")
            except Exception as e:
                print(f"❌ Failed to connect to Kvaser CANlib Channel {channel_num}: {e}")
                return
    elif HAS_PYTHON_CAN:
        try:
            ch = can.Bus(interface='kvaser', channel=channel_num, bitrate=500000)
            print(f"✅ Connected via python-can (Kvaser) Channel {channel_num}")
        except Exception as e:
            print(f"❌ Failed to connect via python-can (Kvaser): {e}")
            return

    # Send CANopen NMT Start Remote Node command
    try:
        if HAS_CANLIB and isinstance(ch, canlib.Channel):
            nmt_frame = Frame(id_=0x000, data=bytearray([0x01, parser.node_id & 0xFF]))
            ch.write(nmt_frame)
        elif HAS_PYTHON_CAN:
            nmt_msg = can.Message(arbitration_id=0x000, data=[0x01, parser.node_id], is_extended_id=False)
            ch.send(nmt_msg)
        print(f"✅ Sent NMT Start Remote Node command to Node ID {parser.node_id}")
    except Exception as e:
        print(f"⚠️ Failed to send NMT Start command: {e}")

    print("-" * 60)
    print(" Listening for incoming GIM700DR CAN messages (Press Ctrl+C to stop)...")
    print("-" * 60)

    try:
        while True:
            can_id = None
            data_bytes = None
            dlc = 0

            if HAS_CANLIB and isinstance(ch, canlib.Channel):
                try:
                    frame = ch.read(timeout=500)
                    can_id = frame.id
                    data_bytes = bytes(frame.data)
                    dlc = len(data_bytes)
                except canlib.CanNoMsg:
                    continue
                except Exception:
                    continue
            elif HAS_PYTHON_CAN:
                msg = ch.recv(timeout=0.5)
                if msg is None:
                    continue
                can_id = msg.arbitration_id
                data_bytes = bytes(msg.data)
                dlc = msg.dlc

            if can_id is not None and data_bytes is not None:
                can_id_hex = hex(can_id)
                data_hex = data_bytes.hex().upper()
                parsed = parser.parse(can_id, data_bytes)

                if parsed:
                    temp = parsed.get('Temperature (°C)', 'N/A')
                    slope_z = parsed.get('Slope Z (°)', 'N/A')
                    slope_y = parsed.get('Slope Y (°)', 'N/A')
                    print(f"[CAN ID: {can_id_hex} | DLC: {dlc} | DATA: {data_hex}] --> Slope Z (X-tilt): {slope_z}°, Slope Y (Z-tilt): {slope_y}°, Temp: {temp}°C")
                else:
                    print(f"[CAN ID: {can_id_hex} | DLC: {dlc} | DATA: {data_hex}]")

    except KeyboardInterrupt:
        print("\nStopping GIM700DR CAN Monitor...")
    finally:
        if HAS_CANLIB and isinstance(ch, canlib.Channel):
            ch.busOff()
            ch.close()
        elif HAS_PYTHON_CAN and ch is not None:
            ch.shutdown()
        print("Kvaser CAN channel closed.")


if __name__ == "__main__":
    run_canlib_cli_monitor()
