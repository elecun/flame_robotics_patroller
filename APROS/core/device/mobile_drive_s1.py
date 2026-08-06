"""
Mobile Drive S1 CAN Hardware Controller Device.
Provides CAN interface for steering angle (-28 to +28 deg) mapped to CAN command (-2000 to +2000).
Protocol referenced from PatrolCar_SlideBar.py (CAN ID 0x502).
"""
import os
import time
import threading
from datetime import datetime
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
                    parsed['vehicle_gear'] = ["P", "D", "N", "R"][vehicle_gear]
                    clamping_brake = (data[0] >> 2) & 0x01
                    parsed['clamping_brake_status'] = "Applied" if clamping_brake else "Released"
                    drive_state_mode = data[1] & 0x03
                    parsed['drive_mode_state_val'] = drive_state_mode
                    parsed['drive_state_mode'] = ["Remote Control Mode", "AD Mode",
                                                  "Parallel Mode", "Semi-autonomous"][drive_state_mode]
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
                    steer_deg = int.from_bytes(data[4:6], 'little') * 0.1 - 35
                    parsed['vehicle_velocity_val'] = float(speed)
                    parsed['vehicle_steer_angle_val'] = float(steer_deg)
                    parsed['vehicle_velocity'] = f"{speed:.1f} km/h"
                    parsed['vehicle_steer_angle'] = f"{steer_deg:.1f} deg"
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
                    bms_soc_val = data[4] * 0.4
                    parsed['bms_battery_soh'] = f"{data[7]} %"
                    parsed['bms_battery_soc'] = f"{bms_soc_val:.2f} %"
                    parsed['bms_battery_voltage'] = f"{int.from_bytes(data[2:4], 'little') * 0.1:.2f} V"
        except Exception as e:
            pass
        return parsed

class MobileDriveS1(BaseDevice, MobileS1API):
    def __init__(
        self,
        name: str = "MobileDriveS1",
        can_channel: int = 0,
        max_steering_angle: float = 30.0,
        min_velocity: float = -1.0,
        max_velocity: float = 5.0,
        lookahead_distance: float = 3.0,
        corridor_boundary: float = 2.5,
        auto_mode_interval_ms: float = 20.0,
        enable: bool = True,
        **kwargs
    ):
        super().__init__(name, enable=enable)
        self.channel = int(can_channel) if isinstance(can_channel, int) or (isinstance(can_channel, str) and str(can_channel).isdigit()) else 0
        self.ch = None
        self.parser = CANParser()

        # Steering angle bounds (degrees) - Default: 30.0 deg (min_steering_angle = -1 * max_steering_angle)
        if "max_steer_angle" in kwargs:
            max_steering_angle = kwargs["max_steer_angle"]
        self.max_steering_angle = abs(float(max_steering_angle))
        self.min_steering_angle = -self.max_steering_angle

        # Velocity bounds (km/h) - soft limit bounds for input velocity control
        self.MIN_VELOCITY_KMH = float(min_velocity)
        self.MAX_VELOCITY_KMH = float(max_velocity)

        # Lookahead distance & corridor boundary for local path planning (meters)
        self.lookahead_distance = float(lookahead_distance)
        self.corridor_boundary = float(corridor_boundary)

        # Auto Mode transmission interval (ms)
        self.auto_mode_interval_ms = float(auto_mode_interval_ms)

        # Command mapping bounds (-2000 ~ +2000)
        # Left: -2000 (at min deg), Right: +2000 (at max deg)
        self.MIN_CMD_VAL = -2000
        self.MAX_CMD_VAL = 2000

        # Initial State (As required: speed=0, steer=0, gear=P, drive_mode=Remote Control Mode)
        self.speed = 0.0  # km/h (monitored/actual status value or current setpoint)
        self.steer_angle = 0.0  # degrees (monitored/actual status value or current setpoint)
        self.cmd_speed = 0.0  # km/h (command input value with soft limit applied)
        self.cmd_steering_angle = 0.0  # degrees (command input value with soft limit applied)
        self.lat = 37.5665
        self.lon = 126.9780
        self.target_gear = "P"  # Commanded Gear (P, D, N, R)
        self.gear = "P"         # Gear Status
        self.drive_mode = "Remote Control Mode"
        self.ad_control_req_flag = 0  # 0: Remote Control Mode, 1: Auto Mode
        self.simulated_heading = 0.0
        self.parsed_can_status = {}

        # Control states for AD Control
        self.cmd_brake = 0.0          # 0.0 ~ 100.0 %
        self.ad_dbs_valid = 0         # 0: Normal Drive, 1: Active Brake Control
        self.left_turn_light = False  # True / False
        self.right_turn_light = False # True / False
        self.head_light = False       # True / False
        self.brake_light = False      # True / False

        # Message Rolling Counters (0..15)
        self.msg_counters = {0x501: 0, 0x502: 0, 0x503: 0, 0x504: 0, 0x506: 0}

        # Periodic 20ms TX & RX Thread Control
        self._tx_running = False
        self._tx_thread = None
        self._rx_running = False
        self._rx_thread = None
        self._ad_tx_lock = threading.Lock()

    def reset_initial_state(self):
        """Reset state to initial values: speed 0, steer 0, gear P, mode Remote Control Mode."""
        self.speed = 0.0
        self.steer_angle = 0.0
        self.target_gear = "P"
        self.gear = "P"
        self.ad_control_req_flag = 0
        self.drive_mode = "Remote Control Mode"

    def _next_cntr(self, can_id: int) -> int:
        cntr = self.msg_counters.get(can_id, 0)
        self.msg_counters[can_id] = (cntr + 1) % 16
        return cntr

    def connect(self) -> bool:
        """Connect to Kvaser CANlib channel 0 in Standard CAN mode (500k) and start 20ms TX/RX threads if enabled."""
        if not self.enable:
            self.is_connected = False
            logger.info(f"[{self.name}] Device is DISABLED in config (enable=False).")
            return False

        self.reset_initial_state()

        if not CANLIB_AVAILABLE or canlib is None:
            self.is_connected = False
            self.ch = None
            logger.warning(f"[{self.name}] CANlib is not available on this system. Operating without physical CAN hardware.")
            return False
        try:
            self.ch = canlib.openChannel(self.channel)
            self.ch.setBusParams(canlib.Bitrate.BITRATE_500K)
            self.ch.busOn()
            self.is_connected = True
            logger.info(f"[{self.name}] Connected to Kvaser CANlib Channel {self.channel} (Standard CAN, 500k). Initialized: Speed=0, Steer=0, Gear=P, Mode=Remote Control.")
            self._start_threads()
            return True
        except canlib.CanError as e:
            self.is_connected = False
            self.ch = None
            logger.error(f"[{self.name}] Failed to connect to Kvaser CANlib Channel {self.channel}: {e}")
            return False
        except Exception as e:
            self.is_connected = False
            self.ch = None
            logger.error(f"[{self.name}] Failed to connect to Kvaser CANlib Channel {self.channel}: {e}")
            return False

    def disconnect(self) -> bool:
        """Reset state to initial values, send final CAN reset frames, and disconnect."""
        logger.info(f"[{self.name}] Disconnecting... Resetting initial values (Speed=0, Steer=0, Gear=P, Mode=Remote Control).")
        self.reset_initial_state()
        if self.is_connected and self.ch is not None and Frame is not None:
            try:
                # Send final reset frames
                self._send_20ms_can_control_frames()
            except Exception:
                pass

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
        """Start RX receiver thread on connection."""
        if not self._rx_running:
            self._rx_running = True
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()

    def _stop_threads(self):
        """Stop AD periodic CAN transmission and RX receiver threads."""
        self._rx_running = False
        self.stop_ad_tx_thread()
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None

    def start_ad_tx_thread(self):
        """Start the AD periodic 20ms CAN transmission thread if not already running."""
        with self._ad_tx_lock:
            if self._tx_running and self._tx_thread and self._tx_thread.is_alive():
                return
            self._tx_running = True
            self._tx_thread = threading.Thread(target=self._periodic_tx_loop, daemon=True)
            self._tx_thread.start()
            logger.info(f"[{self.name}] AD 20ms Periodic CAN TX thread started.")

    def stop_ad_tx_thread(self):
        """Stop the AD periodic CAN transmission thread and send exit flags (AD_Control_Request_Flag=0)."""
        with self._ad_tx_lock:
            if not self._tx_running:
                return
            self._tx_running = False
            thread = self._tx_thread
            self._tx_thread = None

        if thread and thread.is_alive() and threading.current_thread() != thread:
            thread.join(timeout=1.0)

        # Send final exit frames (AD_Control_Request_Flag = 0)
        self.ad_control_req_flag = 0
        if self.is_connected and self.ch is not None and Frame is None:
            pass
        elif self.is_connected and self.ch is not None:
            try:
                self._send_20ms_can_control_frames()
            except Exception:
                pass
        logger.info(f"[{self.name}] AD 20ms Periodic CAN TX thread stopped.")

    def _rx_loop(self):
        """Background thread loop to read and parse incoming CAN 0 messages."""
        logger.info(f"[{self.name}] RX Receiver thread started on CAN Channel {self.channel}...")
        last_can_msg_time = 0.0
        while self._rx_running:
            if not self.is_connected or self.ch is None:
                time.sleep(0.1)
                continue
            try:
                frame = self.ch.read(timeout=100)
                last_can_msg_time = time.time()
                self._has_can_feedback = True

                # Forward 0x301 and 0x303 CAN frames to registered frame recorder (DataLogger)
                if frame.id in (0x301, 0x303) and hasattr(self, 'frame_recorder') and self.frame_recorder is not None:
                    try:
                        self.frame_recorder(frame)
                    except Exception as fe:
                        pass

                parsed = self.parser.parse(frame.id, frame.data)
                if parsed:
                    self.parsed_can_status.update(parsed)
                    if frame.id == 0x303:
                        if "drive_state_mode" in parsed:
                            self.drive_mode = parsed["drive_state_mode"]
                        if "vehicle_gear" in parsed:
                            self.gear = parsed["vehicle_gear"]
                    elif frame.id == 0x304:
                        if "vehicle_velocity_val" in parsed:
                            self.speed = parsed["vehicle_velocity_val"]
                        if "vehicle_steer_angle_val" in parsed:
                            self.steer_angle = parsed["vehicle_steer_angle_val"]
            except canlib.CanNoMsg:
                if time.time() - last_can_msg_time > 2.0:
                    self._has_can_feedback = False
                continue
            except canlib.CanError:
                time.sleep(0.1)
            except Exception:
                time.sleep(0.1)
        logger.info(f"[{self.name}] RX Receiver thread stopped.")

    def _periodic_tx_loop(self):
        """
        Background loop sending 0x501, 0x502, 0x503, 0x504, 0x506 CAN frames
        at a 20ms (50Hz) periodic rate matching PatrolCar_SlideBar_original specification.
        """
        logger.info(f"[{self.name}] 20ms Periodic CAN TX thread active (0x501, 0x502, 0x503, 0x504, 0x506)...")
        while self._tx_running:
            start_time = time.time()
            if self.is_connected and self.ch is not None:
                self._send_20ms_can_control_frames()
            elapsed = time.time() - start_time
            time.sleep(max(0.0, 0.02 - elapsed))

    def _send_20ms_can_control_frames(self):
        """
        Build and send 0x501, 0x502, 0x503, 0x504, 0x506 CAN control frames.
        Reference implementation from PatrolCar_SlideBar_original.py _tick_impl.
        """
        if not self.is_connected or self.ch is None or Frame is None:
            return

        try:
            # 1. 0x501 AD_Control_Flag (Byte0: cntr<<4 | ad_control_req_flag)
            cntr_501 = self._next_cntr(0x501)
            byte0_501 = (cntr_501 << 4) | (self.ad_control_req_flag & 0x1)
            msg_501 = Frame(id_=0x501, data=bytes([byte0_501, 0, 0, 0, 0, 0, 0, 0]))
            self.ch.write(msg_501)

            # 2. 0x502 AD_Control_Steering
            # Byte0: cntr<<4 | (ad_steer_valid & 0x1) -- Start Byte 0, Start Bit 0 (1 when Auto, 0 when Remote)
            # Invert sign (+ is Right turn, - is Left turn for CAN command)
            cntr_502 = self._next_cntr(0x502)
            byte0_502 = (cntr_502 << 4) | (self.ad_control_req_flag & 0x1)
            clamped_angle = max(self.min_steering_angle, min(self.max_steering_angle, float(-self.cmd_steering_angle)))
            raw_angle = int(round((clamped_angle + 30.0) / 0.1))
            raw_angle = max(0, min(0xFFFF, raw_angle))
            data_502 = [byte0_502, 0, 0, 0, raw_angle & 0xFF, (raw_angle >> 8) & 0xFF, 0, 0]
            msg_502 = Frame(id_=0x502, data=bytes(data_502))
            self.ch.write(msg_502)
            logger.info(f"[{self.name}][CAN 0x502 Transmit] target_angle: {self.cmd_steering_angle:.2f} deg, CAN_clamped_angle: {clamped_angle:.2f} deg, CAN_raw_angle_0.1deg: {raw_angle} (0x{raw_angle:04X}), CAN_data_bytes: {list(data_502)}")

            # 3. 0x503 AD_Control_Brake
            # Byte0: cntr<<4 | (ad_dbs_valid & 0x1) -- Start Byte 0, Start Bit 0 (1 when Auto or active brake, 0 when Remote/normal)
            # Byte1: AD_brakePressure_cmd (0~100)
            cntr_503 = self._next_cntr(0x503)
            byte0_503 = (cntr_503 << 4) | (self.ad_control_req_flag & 0x1)
            brake_val = int(round(max(0.0, min(100.0, float(self.cmd_brake)))))
            data_503 = [byte0_503, brake_val, 0, 0, 0, 0, 0, 0]
            msg_503 = Frame(id_=0x503, data=bytes(data_503))
            self.ch.write(msg_503)

            # 4. 0x504 AD_Control_Accelerate
            # Byte0: (cntr<<4) | (ad_accelerate_valid & 0x1)  -- Start Byte 0, Start Bit 0 (1 when Auto, 0 when Remote)
            # Byte2: ad_accelerate_work_mode = 1 (Speed Mode) if Auto else 0
            # Byte3: gear (0: P, 1: D, 2: N, 3: R)
            # Byte4: ad_acc_de = 0
            # Byte5: ad_torque_control = 0
            # Byte6-7: raw_speed
            cntr_504 = self._next_cntr(0x504)
            byte0_504 = (cntr_504 << 4) | (self.ad_control_req_flag & 0x1)
            work_mode = 1 if self.ad_control_req_flag == 1 else 0  # 1: Speed Control Mode
            gear_code = 0  # 0: P Gear, 1: D Gear, 2: N Gear, 3: R Gear
            gear_str = str(self.target_gear).strip().upper()
            if gear_str == "R":
                gear_code = 3
            elif gear_str == "D":
                gear_code = 1
            elif gear_str == "N":
                gear_code = 2
            elif gear_str == "P":
                gear_code = 0
            else:
                if self.cmd_speed < -0.01:
                    gear_code = 3
                elif self.cmd_speed > 0.01:
                    gear_code = 1
                else:
                    gear_code = 0

            raw_accde = 0  # ad_acc_de = 0
            raw_torque = 0 # ad_torque_control = 0
            raw_speed = int(round(abs(float(self.cmd_speed)) / 0.1))
            raw_speed = max(0, min(0xFFFF, raw_speed))
            data_504 = [byte0_504, 0, work_mode, gear_code, raw_accde, raw_torque, raw_speed & 0xFF, (raw_speed >> 8) & 0xFF]
            msg_504 = Frame(id_=0x504, data=bytes(data_504))
            self.ch.write(msg_504)

            # 5. 0x506 AD_Control_Body
            # Byte0: cntr<<4 | (head_light<<3 | right_turn<<1 | left_turn<<0)
            # Byte1: brake_light (1 if P gear (gear_code==0) or ad_dbs_valid==1 or manual brake light else 0)
            cntr_506 = self._next_cntr(0x506)
            light_flags = 0
            if self.head_light:
                light_flags |= 0x08  # Bit 3 (0b1000)
            if self.right_turn_light:
                light_flags |= 0x02  # Bit 1 (0b0010)
            if self.left_turn_light:
                light_flags |= 0x01  # Bit 0 (0b0001)

            is_brake_light_on = (gear_code == 0) or (self.ad_dbs_valid == 1) or self.brake_light
            byte0_506 = (cntr_506 << 4) | (light_flags & 0x0F)
            byte1_506 = 1 if is_brake_light_on else 0
            data_506 = [byte0_506, byte1_506, 0, 0, 0, 0, 0, 0]
            msg_506 = Frame(id_=0x506, data=bytes(data_506))
            self.ch.write(msg_506)

        except canlib.CanError as e:
            if getattr(e, 'status', None) == canlib.ErrorNumber.TXBUFOVRFL or getattr(e, 'param', None) == -13 or "overflow" in str(e).lower():
                pass
            else:
                logger.error(f"[{self.name}] CAN Error sending 20ms frames: {e}")
        except Exception as e:
            logger.error(f"[{self.name}] Unexpected error sending 20ms CAN frames: {e}")

    def set_brake(self, brake_pct: float):
        """Set control target brake percentage (0 ~ 100%)."""
        self.cmd_brake = max(0.0, min(100.0, float(brake_pct)))

    def slow_stop(self):
        """Slow Stop: Set target speed to 0 and explicitly release mechanical brake & brake light."""
        self.ad_dbs_valid = 0
        self.cmd_brake = 0.0
        self.brake_light = False
        self.set_speed(0.0)
        logger.info(f"[{self.name}] Slow stop executed (Speed set to 0.0, Brake & DBS Valid released/OFF).")

    def brake_stop(self):
        """Brake Stop: Set AD_brakePressure_cmd to 10 and ad_dbs_valid to 1."""
        self.ad_dbs_valid = 1
        self.set_brake(10.0)
        logger.info(f"[{self.name}] Brake stop executed (Brake Pressure set to 10%, ad_dbs_valid=1).")

    def set_lights(self, left_turn: Optional[bool] = None, right_turn: Optional[bool] = None, head: Optional[bool] = None, brake: Optional[bool] = None):
        """Set vehicle light control states."""
        if left_turn is not None:
            self.left_turn_light = bool(left_turn)
        if right_turn is not None:
            self.right_turn_light = bool(right_turn)
        if head is not None:
            self.head_light = bool(head)
        if brake is not None:
            self.brake_light = bool(brake)

    def change_drive_mode(self, mode: str):
        """
        Change drive mode (Auto or Remote Control).
        Resets target_gear to 'P' on mode transition.
        Starts 20ms AD CAN TX thread when switching to Auto mode; stops thread when switching to Remote mode.
        """
        clean_mode = str(mode).strip()
        is_auto = clean_mode.lower().startswith("auto")
        self.ad_control_req_flag = 1 if is_auto else 0
        self.drive_mode = "Auto Mode" if is_auto else "Remote Control Mode"
        self.target_gear = "D"  # Reset commanded gear to P on mode transition
        self.gear = "D"
        logger.info(f"[{self.name}] Drive mode changed to '{self.drive_mode}' (AD_Control_Request_Flag={self.ad_control_req_flag}, Target Gear=P)")

        if is_auto:
            self.start_ad_tx_thread()
        else:
            self.stop_ad_tx_thread()

    def set_speed(self, speed_kmh: float):
        """Set control target velocity clamped between min_velocity and max_velocity. Automatically sets target_gear to R if negative or D if positive."""
        clamped_speed = max(self.MIN_VELOCITY_KMH, min(float(speed_kmh), self.MAX_VELOCITY_KMH))
        self.cmd_speed = clamped_speed
        self.speed = clamped_speed
        if clamped_speed < -0.01:
            self.target_gear = "R"
            self.gear = "R"
            self.ad_dbs_valid = 0  # Release active brake flag when moving
        elif clamped_speed > 0.01:
            self.target_gear = "D"
            self.gear = "D"
            self.ad_dbs_valid = 0  # Release active brake flag when moving

    def set_steering_angle(self, angle_deg: float):
        """Set control target steering angle with soft limit applied (must stay within [min_steering_angle, max_steering_angle])."""
        clamped_angle = max(self.min_steering_angle, min(float(angle_deg), self.max_steering_angle))
        self.cmd_steering_angle = clamped_angle
        self.steer_angle = clamped_angle

    def set_mode_remote(self):
        """Switch control mode to Remote Control Mode (ad_control_req_flag=0)."""
        self.change_drive_mode("Remote")

    def set_mode_auto(self):
        """Switch control mode to Auto (ad_control_req_flag=1)."""
        self.change_drive_mode("Auto")

    def degree_to_can_cmd(self, angle_deg: float) -> int:
        """
        Map degree (min_steering_angle to max_steering_angle) to raw CAN command (-2000 to +2000).
        Left: -2000 (at min deg), Right: +2000 (at max deg)
        """
        clamped_deg = max(self.min_steering_angle, min(self.max_steering_angle, float(angle_deg)))
        cmd_val = int(round((clamped_deg / self.max_steering_angle) * self.MAX_CMD_VAL))
        return max(self.MIN_CMD_VAL, min(self.MAX_CMD_VAL, cmd_val))

    def can_cmd_to_degree(self, cmd_val: int) -> float:
        """Map raw CAN command (-2000 to +2000) back to degree (min_steering_angle to max_steering_angle)."""
        clamped_cmd = max(self.MIN_CMD_VAL, min(self.MAX_CMD_VAL, int(cmd_val)))
        return (clamped_cmd / float(self.MAX_CMD_VAL)) * self.max_steering_angle

    def update_simulation_step(self, dt: float = 0.05):
        """Kinematics update step for 3D visualization positioning (+ is Right turn, - is Left turn)."""
        speed_m_s = (self.speed * 1000.0) / 3600.0
        wheelbase = 1.5
        
        effective_steer = -self.steer_angle
        if abs(effective_steer) > 0.01:
            turning_radius = wheelbase / float(np.tan(np.radians(effective_steer)))
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

