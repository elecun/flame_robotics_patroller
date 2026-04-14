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
import threading
from datetime import datetime, timedelta
from typing import Optional, Dict, Any
from util.logger.console import ConsoleLogger


class component(QThread):
    signal_updated = pyqtSignal(dict)
    
    def __init__(self, **kwargs):
        super().__init__()
        self.__console = ConsoleLogger.get_logger()
        self.port = kwargs.get("port", "/dev/ttyACM0") # linux
        self.baudrate = kwargs.get("baudrate", 9600)
        self.utc_offset = kwargs.get("utc_offset", 0)
        self.update_interval_sec = kwargs.get("update_interval_sec", 1.0)
        
        self.serial_connection: Optional[serial.Serial] = None
        self.running = False
        
        # Shared data between receive thread and emit thread
        self._lock = threading.Lock()
        self.latest_data = {}
        
        # Emit thread setup
        self.emit_running = True
        self.emit_thread = threading.Thread(target=self._emit_loop, daemon=True)
        self.emit_thread.start()
        
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
            self.__console.error(f"Serial connection failed: {e}")
            return False
    
    def disconnect_serial(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()

    def quality2str(self, quality: int) -> str:
        quality_map = {
            0: "Invalid",
            1: "3D",
            2: "DGPS",
            4: "RTK Fixed",
            5: "RTK Float",
            6: "Estimated/DR"
        }
        return quality_map.get(quality, "Unknown")
    
    def get_local_datetime(self, utc_time: str, date_str: str = None) -> str:
        if not utc_time:
            return ""
        try:
            # Parsing time
            if "." in utc_time:
                time_part, ms_part = utc_time.split(".")
            else:
                time_part, ms_part = utc_time, "000"
                
            time_part = time_part.zfill(6) # hhmmss
            
            # Parsing date
            if date_str and len(date_str) == 6:
                # date_str is ddmmyy
                date_part = date_str
            else:
                # default to today if date not provided
                date_part = datetime.utcnow().strftime("%d%m%y")
                
            # Create a datetime object
            dt_str = f"{date_part} {time_part}"
            dt = datetime.strptime(dt_str, "%d%m%y %H%M%S")
            
            # Add utc offset
            dt = dt + timedelta(hours=self.utc_offset)
            
            # Format yy-mm-dd hh:mm:ss.millisecond
            return f"{dt.strftime('%y-%m-%d %H:%M:%S')}.{ms_part}"
        except Exception:
            return str(utc_time)

    def __parse_coordinates(self, lat_raw: str, lat_dir: str, lon_raw: str, lon_dir: str) -> tuple:
        """Private helper function to parse raw latitude and longitude strings to decimal."""
        latitude = self.convert_to_decimal_degrees(lat_raw, lat_dir) if lat_raw else None
        longitude = self.convert_to_decimal_degrees(lon_raw, lon_dir) if lon_raw else None
        return latitude, longitude

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
            latitude, longitude = self.__parse_coordinates(
                lat_raw=parts[2], lat_dir=parts[3],
                lon_raw=parts[4], lon_dir=parts[5]
            )
            
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
            latitude, longitude = self.__parse_coordinates(
                lat_raw=parts[3], lat_dir=parts[4],
                lon_raw=parts[5], lon_dir=parts[6]
            )
            
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
                "quality": int(parts[2]) if parts[2] else 0,
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

    def _emit_loop(self):
        """Separate thread for emitting data at regular intervals."""
        while self.emit_running:
            try:
                # Capture connection status
                is_connected = bool(self.serial_connection and self.serial_connection.is_open)
                
                # Fetch latest parsed data
                with self._lock:
                    emit_payload = self.latest_data.copy()
                
                # Add connection status to the payload
                emit_payload["connected"] = is_connected

                # Calculate local_datetime
                utc_time_str = emit_payload.get("utc_time")
                date_str = emit_payload.get("date")
                if utc_time_str:
                    emit_payload["local_datetime"] = self.get_local_datetime(utc_time_str, date_str)

                # Emit data
                self.signal_updated.emit(emit_payload)

            except Exception as e:
                self.__console.error(f"Error in emit loop: {e}")

            # Sleep for the configured interval
            time.sleep(self.update_interval_sec)
    
    def run(self):
        self.running = True

        if not self.connect_serial():
            self.__console.error(f"Failed to connect to serial port {self.port}. Reading loop will wait for reconnection.")
        else:
            self.__console.info(f"RTK GNSS receiver connected on {self.port} at {self.baudrate} baud")
        
        while self.running:
            try:
                if self.serial_connection and self.serial_connection.is_open:
                    if self.serial_connection.in_waiting > 0:
                        line = self.serial_connection.readline().decode('utf-8').strip()
                        if line.startswith('$'):
                            parsed_data = self.parse_nmea_sentence(line)
                            if parsed_data:
                                msg_type = parsed_data.get("message_type")
                                if msg_type == "GGA":
                                    lat = parsed_data.get("latitude")
                                    lon = parsed_data.get("longitude")
                                    quality = parsed_data.get("fix_quality")
                                    utc = self.get_local_datetime(parsed_data.get("utc_time"), parsed_data.get("date"))
                                    #self.__console.info(f"[GNSS GGA] Time: {utc}, Lat: {lat}, Lon: {lon}, Quality: {quality}")
                                elif msg_type == "RMC":
                                    lat = parsed_data.get("latitude")
                                    lon = parsed_data.get("longitude")
                                    speed = parsed_data.get("speed_knots")
                                    utc = self.get_local_datetime(parsed_data.get("utc_time"), parsed_data.get("date"))
                                    #self.__console.info(f"[GNSS RMC] Time: {utc}, Lat: {lat}, Lon: {lon}, Speed: {speed} knots")

                                # Update the latest data safely
                                with self._lock:
                                    self.latest_data.update(parsed_data)
                else:
                    # If disconnected, try to stay alive without blocking
                    time.sleep(0.5)

                time.sleep(0.01)
                
            except serial.SerialException:
                # If serial disconnects unexpectedly
                time.sleep(0.5)
                self.disconnect_serial()
            except Exception as e:
                self.__console.error(f"Error reading serial data: {e}")
                time.sleep(0.1)
        
        self.disconnect_serial()
        self.__console.info("RTK GNSS receiver loop finished")
    
    def stop(self):
        self.__console.info("Stopping GNSS RTK stream...")
        self.running = False
        self.emit_running = False
        self.wait(2000)