"""
Mobile Drive S1 CAN Hardware Controller Device.
Provides CAN interface for steering angle (-28 to +28 deg) mapped to CAN command (-2000 to +2000).
Protocol referenced from PatrolCar_SlideBar.py (CAN ID 0x502).
"""
import time
import threading
from typing import Optional, Dict, Any, List, Callable
import numpy as np
from core.device.base import BaseDevice
try:
    from APROS.core.device.mobile_s1_api import MobileS1API
except ImportError:
    from core.device.mobile_s1_api import MobileS1API
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()

try:
    from canlib import canlib, Frame
    CANLIB_AVAILABLE = True
except (ImportError, Exception, BaseException) as e:
    CANLIB_AVAILABLE = False
    canlib = None
    Frame = None
    logger.error(f"[MobileDriveS1] CANlib (libcanlib.so/dll) is unavailable on this system ({e}). Disabling CAN hardware interface.")



class CANParser:
    """
    Parser for incoming CAN frames based on c.py protocol.
    """
    def parse(self, can_id: int, data: bytes) -> dict:
        parsed = {}
        try:
            if can_id == 0x303:
                if len(data) >= 4:
                    vehicle_gear = data[0] & 0x03
                    parsed['vehicle_gear'] = ["P Gear", "D Gear", "N Gear", "R Gear"][vehicle_gear]
                    drive_state_mode = data[1] & 0x03
                    parsed['drive_mode_state_val'] = drive_state_mode
                    parsed['drive_state_mode'] = ["Remote Control Mode", "Represents the AD Mode",
                                                  "Indicates parallel Mode", "Indicates semi-autonomous"][drive_state_mode]
                    vcu_speed_req = int.from_bytes(data[2:4], byteorder='little') * 0.1 - 80
                    parsed['vehicle_speed_request'] = f"{vcu_speed_req:.1f} km/h"

            elif can_id == 0x314:
                if len(data) >= 3:
                    directional_angle = int.from_bytes(data[1:3], byteorder='little')
                    parsed['direction_angle'] = f"{directional_angle} deg"
                    parsed['eps_control'] = "Works" if data[0] & 0x01 else "Stops"

            elif can_id == 0x304:
                if len(data) >= 6:
                    speed = int.from_bytes(data[0:2], byteorder='little') * 0.1 - 80
                    parsed['vehicle_velocity'] = f"{speed:.1f} km/h"
                    parsed['vehicle_steer_angle'] = f"{int.from_bytes(data[4:6], 'little') * 0.1 - 35:.1f} deg"
                    parsed['vehicle_brake_pressure'] = f"{int.from_bytes(data[2:4], 'little') * 0.01:.2f} Mps"

            elif can_id == 0x301:
                if len(data) >= 6:
                    parsed['brake_light'] = "ON" if data[5] & 0x01 else "OFF"
                    parsed['head_light'] = "ON" if data[1] & 0x80 else "OFF"
                    parsed['emergency_button'] = "Pressed" if data[0] & 0x01 else "Not Pressed"
                    parsed['back_touch_switch'] = "trigger" if data[1] & 0x20 else "Not trigger"
                    parsed['front_touch_switch'] = "trigger" if data[1] & 0x10 else "Not trigger"

            elif can_id == 0x18F:
                if len(data) >= 7:
                    parsed['eps_current_angle'] = f"{int.from_bytes(data[1:3], 'little', signed=True)} deg"
                    parsed['eps_ecu_temperature'] = f"{int.from_bytes(data[6:7], 'little', signed=True)} degC"

            elif can_id == 0x060:
                if len(data) >= 4:
                    parsed['bus_voltage'] = f"{int.from_bytes(data[0:2], 'little') * 0.1:.2f} V"
                    parsed['bus_current'] = f"{int.from_bytes(data[2:4], 'little') * 0.1 - 1000:.2f} A"

            elif can_id == 0x160:
                if len(data) >= 6:
                    mode = (data[0] & 0x06) >> 1
                    parsed['mcu_drive_mode'] = ["Torque", "Speed", "Torque ring", "Speed loop"][mode]
                    parsed['mcu_brake_request'] = "Hold brake" if data[0] & 0x08 else "Release"
                    parsed['mcu_speed_request'] = f"{int.from_bytes(data[3:6], 'little') - 7000} RPM"
                    parsed['mcu_torque_request'] = f"{int.from_bytes(data[1:3], 'little') * 0.1 - 1000:.1f} Nm"

            elif can_id == 0x0A0:
                if len(data) >= 8:
                    parsed['bms_battery_soh'] = f"{data[7]} %"
                    parsed['bms_battery_soc'] = f"{data[4] * 0.4:.2f} %"
                    parsed['bms_battery_voltage'] = f"{int.from_bytes(data[2:4], 'little') * 0.1:.2f} V"
        except Exception as e:
            pass
        return parsed

class MobileDriveS1(BaseDevice, MobileS1API):
    def __init__(
        self,
        name: str = "MobileDriveS1",
        can_channel: int = 0,
        min_steer_angle: float = -28.0,
        max_steer_angle: float = 28.0,
        max_velocity: float = 5.0,
        lookahead_distance: float = 3.0,
        auto_mode_interval_ms: float = 20.0,
        status_monitor: Optional[Any] = None,
        enable: bool = True,
        **kwargs
    ):
        super().__init__(name, enable=enable, status_monitor=status_monitor)
        self.channel = int(can_channel) if isinstance(can_channel, int) or (isinstance(can_channel, str) and str(can_channel).isdigit()) else 0
        self.ch = None
        self.parser = CANParser()

        # Steering angle bounds (degrees)
        self.MIN_ANGLE_DEG = float(min_steer_angle)
        self.MAX_ANGLE_DEG = float(max_steer_angle)

        # Maximum velocity (km/h)
        self.MAX_VELOCITY_KMH = float(max_velocity)

        # Lookahead distance for local path planning (meters)
        self.lookahead_distance = float(lookahead_distance)

        # Auto Mode transmission interval (ms)
        self.auto_mode_interval_ms = float(auto_mode_interval_ms)

        # Command mapping bounds (-2000 ~ +2000)
        # Left: -2000 (at min deg), Right: +2000 (at max deg)
        self.MIN_CMD_VAL = -2000
        self.MAX_CMD_VAL = 2000

        # Current State
        self.speed = 0.0  # km/h
        self.steer_angle = 0.0  # degrees
        self.lat = 37.5665
        self.lon = 126.9780
        self.gear = "P"
        self.drive_mode = "Manual (Remote)"
        self.simulated_heading = 0.0
        self.parsed_can_status = {}

        # Periodic TX & RX Thread Control
        self._tx_running = False
        self._tx_thread = None
        self._rx_running = False
        self._rx_thread = None

        # Auto Mode Thread Control
        self._auto_tx_running = False
        self._auto_tx_thread = None
        self._auto_tx_lock = threading.Lock()

    def set_auto_mode_interval(self, interval_ms: float):
        """Set auto mode CAN transmission interval in milliseconds."""
        self.auto_mode_interval_ms = float(interval_ms)

    def connect(self) -> bool:
        """Connect to Kvaser CANlib channel 0 in Standard CAN mode (500k) and start TX/RX threads if enabled."""
        if not self.enable:
            self.is_connected = False
            logger.info(f"[{self.name}] Device is DISABLED in config (enable=False).")
            return False

        if not CANLIB_AVAILABLE or canlib is None:
            self.is_connected = False
            self.ch = None
            logger.warning(f"[{self.name}] CANlib is not available on this system. Operating without physical CAN hardware.")
            return False
        try:

            self.ch = canlib.openChannel(self.channel, flags=canlib.Open.ACCEPT_VIRTUAL)
            self.ch.setBusParams(canlib.Bitrate.BITRATE_500K)
            self.ch.busOn()
            self.is_connected = True
            logger.info(f"[{self.name}] Connected to Kvaser CANlib Channel {self.channel} (Standard CAN, 500k).")
            self._start_threads()
            return True
        except canlib.CanError as e:
            try:
                self.ch = canlib.openChannel(self.channel)
                self.ch.setBusParams(canlib.Bitrate.BITRATE_500K)
                self.ch.busOn()
                self.is_connected = True
                logger.info(f"[{self.name}] Connected to Kvaser CANlib Channel {self.channel} (Standard CAN, 500k).")
                self._start_threads()
                return True
            except Exception as ex:
                self.is_connected = False
                self.ch = None
                logger.error(f"[{self.name}] Failed to connect to Kvaser CANlib Channel {self.channel}: {ex}")
                return False
        except Exception as e:
            self.is_connected = False
            self.ch = None
            logger.error(f"[{self.name}] Failed to connect to Kvaser CANlib Channel {self.channel}: {e}")
            return False

    def disconnect(self) -> bool:
        """Disconnect Kvaser CANlib interface and stop threads."""
        self.set_mode_manual()
        self._stop_threads()
        if self.ch is not None:
            try:
                self.ch.busOff()
                self.ch.close()
            except Exception:
                pass
            self.ch = None
        self.is_connected = False
        logger.info(f"[{self.name}] Disconnected from Kvaser CAN bus.")
        return True

    def _start_threads(self):
        """Start 100ms periodic CAN transmission thread and RX receiver thread."""
        if not self._tx_running:
            self._tx_running = True
            self._tx_thread = threading.Thread(target=self._periodic_tx_loop, daemon=True)
            self._tx_thread.start()
        if not self._rx_running:
            self._rx_running = True
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()

    def _stop_threads(self):
        """Stop periodic CAN transmission and RX receiver threads."""
        self._tx_running = False
        self._rx_running = False
        self.stop_auto_mode_thread()
        if self._tx_thread:
            self._tx_thread.join(timeout=1.0)
            self._tx_thread = None
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None

    def _rx_loop(self):
        """Background thread loop to read and parse incoming CAN 0 messages."""
        logger.info(f"[{self.name}] RX Receiver thread started on CAN Channel {self.channel}...")
        while self._rx_running:
            if not self.is_connected or self.ch is None:
                time.sleep(0.1)
                continue
            try:
                frame = self.ch.read(timeout=100)
                parsed = self.parser.parse(frame.id, frame.data)
                if parsed:
                    # Update parsed CAN status storage quietly
                    self.parsed_can_status.update(parsed)
                    
                    # Stop Auto mode periodic TX thread if VCU reports Drive_Mode_State == 1 (Represents the AD Mode)
                    if frame.id == 0x303 and parsed.get("drive_mode_state_val") == 1:
                        if self._auto_tx_running:
                            logger.info(f"[{self.name}] Received VCU_Vehicle_Status_1 (0x303) Drive_Mode_State == 1 (AD Mode Active). Stopping Auto periodic TX thread.")
                            self.stop_auto_mode_thread()
            except canlib.CanNoMsg:
                continue
            except canlib.CanError:
                time.sleep(0.1)
            except Exception:
                time.sleep(0.1)
        logger.info(f"[{self.name}] RX Receiver thread stopped.")

    def _periodic_tx_loop(self):
        """
        Background loop sending steering angle (0x502) and drive speed (0x504) CAN frames
        at a 100ms (10Hz) periodic rate.
        """
        while self._tx_running:
            start_time = time.time()
            if self.is_connected and self.ch is not None:
                self._send_can_control_frames()
            elapsed = time.time() - start_time
            time.sleep(max(0.0, 0.1 - elapsed))

    def _send_can_control_frames(self):
        """Build and send current steering angle and speed CAN control frames using Standard CAN via S1 API functions."""
        # 1. Steering Angle Frame (0x502)
        clamped_angle = max(self.MIN_ANGLE_DEG, min(self.MAX_ANGLE_DEG, float(self.steer_angle)))
        steer_payload = self._s1_api_ad_control_steering(
            ad_steering_valid=0xF1,
            ad_steering_angle_cmd=clamped_angle
        )

        # 2. Drive Speed & Gear Frame (0x504)
        gear_code = 0x02  # N
        if self.gear == "P":
            gear_code = 0x00
        elif self.gear == "D":
            gear_code = 0x01
        elif self.gear == "N":
            gear_code = 0x02
        elif self.gear == "R":
            gear_code = 0x03

        speed_payload = self._s1_api_ad_control_accelerate(
            ad_accelerate_valid=0xF1,
            ad_accelerate_work_mode=0x00,
            ad_accelerate_gear=gear_code,
            ad_speed_control=abs(self.speed)
        )

        if not self.is_connected or self.ch is None or Frame is None:
            return

        try:
            # Standard CAN Frames (0x502 & 0x504)
            frame_502 = Frame(id_=0x502, data=steer_payload)
            frame_504 = Frame(id_=0x504, data=speed_payload)
            self.ch.write(frame_502)
            self.ch.write(frame_504)

        except canlib.CanError as e:
            if getattr(e, 'status', None) == canlib.ErrorNumber.TXBUFOVRFL or getattr(e, 'param', None) == -13:
                pass
            else:
                logger.error(f"[{self.name}] CAN Error sending frames: {e}")
        except Exception as e:
            logger.error(f"[{self.name}] Unexpected error sending CAN frames: {e}")

    def degree_to_can_cmd(self, angle_deg: float) -> int:
        """
        Map degree (-28.0 to +28.0) to raw CAN command (-2000 to +2000).
        Left: -2000 (at -28 deg), Right: +2000 (at +28 deg)
        """
        clamped_deg = max(self.MIN_ANGLE_DEG, min(self.MAX_ANGLE_DEG, float(angle_deg)))
        cmd_val = int(round((clamped_deg / self.MAX_ANGLE_DEG) * self.MAX_CMD_VAL))
        return max(self.MIN_CMD_VAL, min(self.MAX_CMD_VAL, cmd_val))

    def _send_ad_control_flag_frame(self, request_flag: int):
        """
        Send CAN frame 0x501 (AD_Control_Flag).
        Byte 0 header: 0xF1 for Auto mode request (0x01), 0xF0 for exiting Auto mode (0x00).
        """
        first_byte = 0xF1 if (request_flag & 0xFF) == 1 else 0xF0
        payload = self._s1_api_ad_control_flag(ad_control_request_flag=first_byte)

        if not self.is_connected or self.ch is None or Frame is None:
            return

        try:
            frame_501 = Frame(id_=0x501, data=payload)
            self.ch.write(frame_501)
        except canlib.CanError as e:
            if getattr(e, 'status', None) == canlib.ErrorNumber.TXBUFOVRFL or getattr(e, 'param', None) == -13:
                pass
            else:
                logger.error(f"[{self.name}] CAN Error sending 0x501 frame: {e}")
        except Exception as e:
            logger.error(f"[{self.name}] Unexpected error sending 0x501 CAN frame: {e}")

    def _send_ad_control_accelerate_frame(self, valid_flag: int):
        """
        Send CAN frame 0x504 (AD_Control_Accelerate).
        Byte 0 header: 0xF1 for AD_Accelerate_Valid = 0x01 (Auto mode), 0xF0 for exiting Auto mode (0x00).
        """
        first_byte = 0xF1 if (valid_flag & 0xFF) == 1 else 0xF0
        payload = self._s1_api_ad_control_accelerate(ad_accelerate_valid=first_byte)

        if not self.is_connected or self.ch is None or Frame is None:
            return

        try:
            frame_504 = Frame(id_=0x504, data=payload)
            self.ch.write(frame_504)
        except canlib.CanError as e:
            if getattr(e, 'status', None) == canlib.ErrorNumber.TXBUFOVRFL or getattr(e, 'param', None) == -13:
                pass
            else:
                logger.error(f"[{self.name}] CAN Error sending 0x504 frame: {e}")
        except Exception as e:
            logger.error(f"[{self.name}] Unexpected error sending 0x504 CAN frame: {e}")

    def _auto_mode_tx_loop(self):
        """
        Periodic thread loop for Auto mode.
        Sends CAN ID 0x501 (AD_Control_Flag) and 0x503 (AD_Control_Accelerate) at configured interval (default 20ms).
        """
        logger.info(f"[{self.name}] Auto Mode CAN TX thread started (Interval: {self.auto_mode_interval_ms} ms)")
        while self._auto_tx_running:
            start_time = time.time()
            self._send_ad_control_flag_frame(0x01)
            self._send_ad_control_accelerate_frame(0x01)
            elapsed = time.time() - start_time
            interval_sec = max(0.001, self.auto_mode_interval_ms / 1000.0)
            time.sleep(max(0.0, interval_sec - elapsed))
        
        # Send 0x00 / 0xF0 flag when exiting Auto Mode
        self._send_ad_control_flag_frame(0x00)
        self._send_ad_control_accelerate_frame(0x00)
        logger.info(f"[{self.name}] Auto Mode CAN TX thread stopped (Sent exit flags 0xF0)")

    def start_auto_mode_thread(self):
        """Start the Auto mode CAN transmission thread if not already running."""
        with self._auto_tx_lock:
            if self._auto_tx_running and self._auto_tx_thread and self._auto_tx_thread.is_alive():
                logger.info(f"[{self.name}] Auto mode thread is already running.")
                return
            self._auto_tx_running = True
            self._auto_tx_thread = threading.Thread(target=self._auto_mode_tx_loop, daemon=True)
            self._auto_tx_thread.start()

    def stop_auto_mode_thread(self):
        """Stop the Auto mode CAN transmission thread if running."""
        with self._auto_tx_lock:
            if not self._auto_tx_running:
                return
            self._auto_tx_running = False
            thread = self._auto_tx_thread
            self._auto_tx_thread = None

        if thread and thread.is_alive() and threading.current_thread() != thread:
            thread.join(timeout=1.0)

    def can_cmd_to_degree(self, cmd_val: int) -> float:
        """Map raw CAN command (-2000 to +2000) back to degree (-28.0 to +28.0)."""
        clamped_cmd = max(self.MIN_CMD_VAL, min(self.MAX_CMD_VAL, int(cmd_val)))
        return (clamped_cmd / float(self.MAX_CMD_VAL)) * self.MAX_ANGLE_DEG

    def set_steering_angle(self, angle_deg: float):
        """Update target steering angle in degrees."""
        clamped_angle = max(self.MIN_ANGLE_DEG, min(self.MAX_ANGLE_DEG, float(angle_deg)))
        self.steer_angle = clamped_angle

    def change_drive_mode(self, mode: str):
        """Change drive mode (Auto or Manual). Log action to console and manage Auto thread/timer."""
        clean_mode = str(mode).strip()
        is_target_auto = clean_mode.lower().startswith("auto")
        is_current_auto = self.drive_mode.lower().startswith("auto")

        if is_target_auto == is_current_auto and (self._auto_tx_running if is_target_auto else not self._auto_tx_running):
            logger.info(f"[{self.name}] Drive mode already set to '{clean_mode}'. Ignoring duplicate mode switch request.")
            return

        self.drive_mode = clean_mode
        logger.info(f"[{self.name}] change_drive_mode('{clean_mode}') executed.")
        if is_target_auto:
            self.start_auto_mode_thread()
        else:
            self.stop_auto_mode_thread()

    def set_mode_manual(self):
        """Switch control mode to Manual and stop Auto thread."""
        self.change_drive_mode("Manual")

    def set_mode_auto(self):
        """Switch control mode to Auto and start Auto thread."""
        self.change_drive_mode("Auto")

    def update_simulation_step(self, dt: float = 0.05):
        """Kinematics update step for 3D visualization positioning."""
        speed_m_s = (self.speed * 1000.0) / 3600.0
        wheelbase = 1.5
        
        if abs(self.steer_angle) > 0.01:
            turning_radius = wheelbase / float(np.tan(np.radians(self.steer_angle)))
            angular_velocity = speed_m_s / turning_radius
        else:
            angular_velocity = 0.0

        self.simulated_heading += angular_velocity * dt
        d_lat = (speed_m_s * dt * float(np.cos(self.simulated_heading))) / 111000.0
        d_lon = (speed_m_s * dt * float(np.sin(self.simulated_heading))) / (111000.0 * 0.79)
        self.lat += d_lat
        self.lon += d_lon

    def get_status(self) -> dict:
        status_dict = {
            "name": self.name,
            "channel": self.channel,
            "connected": self.is_connected,
            "speed": self.speed,
            "steer_angle": self.steer_angle,
            "vehicle_velocity": f"{self.speed:.1f} km/h",
            "vehicle_steer_angle": f"{self.steer_angle:.1f} deg",
            "can_cmd_val": self.degree_to_can_cmd(self.steer_angle),
            "latitude": self.lat,
            "longitude": self.lon,
            "gear": self.gear,
            "drive_mode": self.drive_mode,
            "heading": self.simulated_heading,
            "parsed_can_status": self.parsed_can_status
        }
        return status_dict

