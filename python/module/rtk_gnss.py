"""
RTK GNSS Module for SMC2000 RTK Receiver
@author Byunghun Hwang<bh.hwang@iae.re.kr>
"""

try:
    from PyQt6.QtCore import QThread, pyqtSignal, QTimer
    from PyQt6.QtWidgets import QApplication
except ImportError:
    print("PyQt6 is required to run this module.")

import serial
import json
import time
import re
from typing import Optional, Dict, Any


class smc2000_rtk(QThread):
    data_received = pyqtSignal(str)
    
    def __init__(self, port: str, baudrate: int = 9600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.serial_connection: Optional[serial.Serial] = None
        self.running = False
        
    def connect_serial(self) -> bool:
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            return self.serial_connection.is_open
        except Exception as e:
            print(f"Serial connection failed: {e}")
            return False
    
    def disconnect_serial(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
    
    def parse_nmea_sentence(self, sentence: str) -> Dict[str, Any]:
        sentence = sentence.strip()
        if not sentence.startswith('$'):
            return {}
        
        parts = sentence.split(',')
        if len(parts) < 3:
            return {}
        
        sentence_id = parts[0][1:]
        parsed_data = {
            "sentence_id": sentence_id,
            "raw": sentence,
            "timestamp": time.time()
        }
        
        if sentence_id.endswith('GGA'):
            parsed_data.update(self.parse_gga(parts))
        elif sentence_id.endswith('RMC'):
            parsed_data.update(self.parse_rmc(parts))
        elif sentence_id.endswith('GSA'):
            parsed_data.update(self.parse_gsa(parts))
        elif sentence_id.endswith('GSV'):
            parsed_data.update(self.parse_gsv(parts))
        
        return parsed_data
    
    def parse_gga(self, parts: list) -> Dict[str, Any]:
        if len(parts) < 15:
            return {}
        
        try:
            lat_raw = parts[2]
            lat_dir = parts[3]
            lon_raw = parts[4]
            lon_dir = parts[5]
            
            latitude = self.convert_to_decimal_degrees(lat_raw, lat_dir) if lat_raw else None
            longitude = self.convert_to_decimal_degrees(lon_raw, lon_dir) if lon_raw else None
            
            return {
                "message_type": "GGA",
                "utc_time": parts[1],
                "latitude": latitude,
                "longitude": longitude,
                "fix_quality": int(parts[6]) if parts[6] else 0,
                "num_satellites": int(parts[7]) if parts[7] else 0,
                "hdop": float(parts[8]) if parts[8] else None,
                "altitude": float(parts[9]) if parts[9] else None,
                "altitude_units": parts[10],
                "geoid_height": float(parts[11]) if parts[11] else None,
                "geoid_units": parts[12],
                "dgps_age": float(parts[13]) if parts[13] else None,
                "dgps_station": parts[14].split('*')[0] if parts[14] else None
            }
        except (ValueError, IndexError):
            return {"message_type": "GGA", "parse_error": True}
    
    def parse_rmc(self, parts: list) -> Dict[str, Any]:
        if len(parts) < 12:
            return {}
        
        try:
            lat_raw = parts[3]
            lat_dir = parts[4]
            lon_raw = parts[5]
            lon_dir = parts[6]
            
            latitude = self.convert_to_decimal_degrees(lat_raw, lat_dir) if lat_raw else None
            longitude = self.convert_to_decimal_degrees(lon_raw, lon_dir) if lon_raw else None
            
            return {
                "message_type": "RMC",
                "utc_time": parts[1],
                "status": parts[2],
                "latitude": latitude,
                "longitude": longitude,
                "speed_knots": float(parts[7]) if parts[7] else None,
                "track_angle": float(parts[8]) if parts[8] else None,
                "date": parts[9],
                "magnetic_variation": float(parts[10]) if parts[10] else None,
                "variation_direction": parts[11].split('*')[0] if parts[11] else None
            }
        except (ValueError, IndexError):
            return {"message_type": "RMC", "parse_error": True}
    
    def parse_gsa(self, parts: list) -> Dict[str, Any]:
        if len(parts) < 18:
            return {}
        
        try:
            satellites = []
            for i in range(3, 15):
                if parts[i]:
                    satellites.append(int(parts[i]))
            
            return {
                "message_type": "GSA",
                "mode": parts[1],
                "fix_type": int(parts[2]) if parts[2] else 0,
                "satellites": satellites,
                "pdop": float(parts[15]) if parts[15] else None,
                "hdop": float(parts[16]) if parts[16] else None,
                "vdop": float(parts[17].split('*')[0]) if parts[17] else None
            }
        except (ValueError, IndexError):
            return {"message_type": "GSA", "parse_error": True}
    
    def parse_gsv(self, parts: list) -> Dict[str, Any]:
        if len(parts) < 4:
            return {}
        
        try:
            satellites = []
            for i in range(4, len(parts), 4):
                if i + 3 < len(parts):
                    sat_info = {
                        "prn": int(parts[i]) if parts[i] else None,
                        "elevation": int(parts[i+1]) if parts[i+1] else None,
                        "azimuth": int(parts[i+2]) if parts[i+2] else None,
                        "snr": int(parts[i+3].split('*')[0]) if parts[i+3] and parts[i+3].split('*')[0] else None
                    }
                    satellites.append(sat_info)
            
            return {
                "message_type": "GSV",
                "total_messages": int(parts[1]) if parts[1] else 0,
                "message_number": int(parts[2]) if parts[2] else 0,
                "total_satellites": int(parts[3]) if parts[3] else 0,
                "satellites": satellites
            }
        except (ValueError, IndexError):
            return {"message_type": "GSV", "parse_error": True}
    
    def convert_to_decimal_degrees(self, coord: str, direction: str) -> float:
        if not coord or len(coord) < 4:
            return 0.0
        
        if '.' in coord:
            dot_index = coord.index('.')
            if dot_index >= 3:
                degrees = float(coord[:dot_index-2])
                minutes = float(coord[dot_index-2:])
            else:
                degrees = 0.0
                minutes = float(coord)
        else:
            if len(coord) >= 4:
                degrees = float(coord[:-2])
                minutes = float(coord[-2:])
            else:
                degrees = 0.0
                minutes = float(coord)
        
        decimal_degrees = degrees + minutes / 60.0
        
        if direction in ['S', 'W']:
            decimal_degrees = -decimal_degrees
        
        return decimal_degrees
    
    def run(self):
        if not self.connect_serial():
            print(f"Failed to connect to serial port {self.port}")
            return
        
        self.running = True
        print(f"RTK GNSS receiver connected on {self.port} at {self.baudrate} baud")
        
        while self.running:
            try:
                if self.serial_connection and self.serial_connection.in_waiting > 0:
                    line = self.serial_connection.readline().decode('utf-8').strip()
                    if line.startswith('$'):
                        parsed_data = self.parse_nmea_sentence(line)
                        if parsed_data:
                            json_data = json.dumps(parsed_data, default=str)
                            self.data_received.emit(json_data)
                
                self.msleep(10)
                
            except Exception as e:
                print(f"Error reading serial data: {e}")
                self.msleep(100)
        
        self.disconnect_serial()
        print("RTK GNSS receiver disconnected")
    
    def stop(self):
        self.running = False
        self.wait()