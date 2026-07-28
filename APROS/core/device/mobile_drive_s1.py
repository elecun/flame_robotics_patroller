"""
Mobile Drive S1 CAN Hardware Controller Device.
Provides CAN interface for steering angle (-28 to +28 deg) mapped to CAN command (-2000 to +2000).
Protocol referenced from PatrolCar_SlideBar.py (CAN ID 0x502).
"""
import time
import threading
import numpy as np
from canlib import canlib, Frame
from core.device.base import BaseDevice

class CANParser:
    """
    Parser for incoming CAN frames based on c.py protocol.
    """
    def parse(self, can_id: int, data: bytes) -> dict:
        parsed = {}
        try:
            if can_id == 0x303:
                if len(data) >= 4:
                    # show low data in hexa decimal
                    vehicle_gear = data[0] & 0x03
                    parsed['Vehicle Gear'] = ["P Gear", "D Gear", "N Gear", "R Gear"][vehicle_gear]
                    drive_state_mode = data[1] & 0x03
                    parsed['Drive_State_Mode'] = ["Remote Control Mode", "Represents the AD Mode",
                                                  "Indicates parallel Mode", "Indicates semi-autonomous"][drive_state_mode]
                    vcu_speed_req = int.from_bytes(data[2:4], byteorder='little') * 0.1 - 80
                    parsed['Vehicle Speed Request (km/h)'] = f"{vcu_speed_req:.1f}"

            elif can_id == 0x314:
                if len(data) >= 3:
                    directional_angle = int.from_bytes(data[1:3], byteorder='little')
                    parsed['Direction Angle (deg)'] = f"{directional_angle}"
                    parsed['eps Control'] = "Works" if data[0] & 0x01 else "Stops"

            elif can_id == 0x304:
                if len(data) >= 6:
                    speed = int.from_bytes(data[0:2], byteorder='little') * 0.1 - 80
                    parsed['Vehicle Speed (km/h)'] = f"{speed:.1f}"
                    parsed['Vehicle Steer Angle (deg)'] = f"{int.from_bytes(data[4:6], 'little') * 0.1 - 35:.1f}"
                    parsed['Vehicle Break Pressure (Mps)'] = f"{int.from_bytes(data[2:4], 'little') * 0.01:.2f}"

            elif can_id == 0x301:
                if len(data) >= 6:
                    parsed['Brake Light'] = "ON" if data[5] & 0x01 else "OFF"
                    parsed['Head Light'] = "ON" if data[1] & 0x80 else "OFF"
                    parsed['Emergency Button'] = "Pressed" if data[0] & 0x01 else "Not Pressed"
                    parsed['Back Touch Switch State'] = "trigger" if data[1] & 0x20 else "Not trigger"
                    parsed['Front Touch Switch State'] = "trigger" if data[1] & 0x10 else "Not trigger"

            elif can_id == 0x18F:
                if len(data) >= 7:
                    parsed['EPS_Current_Angle (deg)'] = str(int.from_bytes(data[1:3], 'little', signed=True))
                    parsed['EPS_ECU_Temperature (℃)'] = str(int.from_bytes(data[6:7], 'little', signed=True))

            elif can_id == 0x060:
                if len(data) >= 4:
                    parsed['BUS Voltage (V)'] = f"{int.from_bytes(data[0:2], 'little') * 0.1:.2f}"
                    parsed['BUS Current (A)'] = f"{int.from_bytes(data[2:4], 'little') * 0.1 - 1000:.2f}"

            elif can_id == 0x160:
                if len(data) >= 6:
                    mode = (data[0] & 0x06) >> 1
                    parsed['Drive Mode'] = ["Torque", "Speed", "Torque ring", "Speed loop"][mode]
                    parsed['MCU_Brake_Request'] = "Hold brake" if data[0] & 0x08 else "Release"
                    parsed['MCU Speed Request (RPM)'] = str(int.from_bytes(data[3:6], 'little') - 7000)
                    parsed['MCU Torque Request (Nm)'] = f"{int.from_bytes(data[1:3], 'little') * 0.1 - 1000:.1f}"

            elif can_id == 0x0A0:
                if len(data) >= 8:
                    parsed['BMS Battery SOH (%)'] = str(data[7])
                    parsed['BMS Battery SOC (%)'] = f"{data[4] * 0.4:.2f}"
                    parsed['BMS Battery Voltage (V)'] = f"{int.from_bytes(data[2:4], 'little') * 0.1:.2f}"
        except Exception as e:
            pass
        return parsed

class MobileDriveS1(BaseDevice):
    def __init__(
        self,
        name: str = "MobileDriveS1",
        can_channel: int = 0,
        min_steer_angle: float = -28.0,
        max_steer_angle: float = 28.0,
        max_velocity: float = 5.0
    ):
        super().__init__(name)
        self.channel = int(can_channel) if isinstance(can_channel, int) or (isinstance(can_channel, str) and str(can_channel).isdigit()) else 0
        self.ch = None
        self.parser = CANParser()

        # Steering angle bounds (degrees)
        self.MIN_ANGLE_DEG = float(min_steer_angle)
        self.MAX_ANGLE_DEG = float(max_steer_angle)

        # Maximum velocity (km/h)
        self.MAX_VELOCITY_KMH = float(max_velocity)

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

    def connect(self) -> bool:
        """Connect to Kvaser CANlib channel 0 in Standard CAN mode (500k) and start TX/RX threads."""
        try:
            self.ch = canlib.openChannel(self.channel, flags=canlib.Open.ACCEPT_VIRTUAL)
            self.ch.setBusParams(canlib.Bitrate.BITRATE_500K)
            self.ch.busOn()
            self.is_connected = True
            print(f"[{self.name}] Connected to Kvaser CANlib Channel {self.channel} (Standard CAN, 500k).")
            self._start_threads()
            return True
        except canlib.CanError as e:
            try:
                self.ch = canlib.openChannel(self.channel)
                self.ch.setBusParams(canlib.Bitrate.BITRATE_500K)
                self.ch.busOn()
                self.is_connected = True
                print(f"[{self.name}] Connected to Kvaser CANlib Channel {self.channel} (Standard CAN, 500k).")
                self._start_threads()
                return True
            except Exception as ex:
                self.is_connected = False
                self.ch = None
                print(f"[{self.name}] Failed to connect to Kvaser CANlib Channel {self.channel}: {ex}")
                return False
        except Exception as e:
            self.is_connected = False
            self.ch = None
            print(f"[{self.name}] Failed to connect to Kvaser CANlib Channel {self.channel}: {e}")
            return False

    def disconnect(self) -> bool:
        """Disconnect Kvaser CANlib interface and stop threads."""
        self._stop_threads()
        if self.ch is not None:
            try:
                self.ch.busOff()
                self.ch.close()
            except Exception:
                pass
            self.ch = None
        self.is_connected = False
        print(f"[{self.name}] Disconnected from Kvaser CAN bus.")
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
        if self._tx_thread:
            self._tx_thread.join(timeout=1.0)
            self._tx_thread = None
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None

    def _rx_loop(self):
        """Background thread loop to read and parse incoming CAN 0 messages."""
        print(f"[{self.name}] RX Receiver thread started on CAN Channel {self.channel}...")
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
            except canlib.CanNoMsg:
                continue
            except canlib.CanError:
                time.sleep(0.1)
            except Exception:
                time.sleep(0.1)
        print(f"[{self.name}] RX Receiver thread stopped.")

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
        """Build and send current steering angle and speed CAN control frames using Standard CAN."""
        # 1. Steering Angle Frame (0x502)
        clamped_angle = max(self.MIN_ANGLE_DEG, min(self.MAX_ANGLE_DEG, float(self.steer_angle)))
        raw_cmd = self.degree_to_can_cmd(clamped_angle)
        unsigned_val = raw_cmd & 0xFFFF
        angular_v1 = unsigned_val & 0xFF
        angular_v2 = (unsigned_val >> 8) & 0xFF

        steer_payload = bytearray([0xF1, 0, 0, 0, angular_v1, angular_v2, 0, 0])

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

        speed_val = int(abs(self.speed) / 0.1)  # 0.1 km/h resolution
        linear_v1 = speed_val & 0xFF
        linear_v2 = (speed_val >> 8) & 0xFF

        speed_payload = bytearray([0xF1, 0x00, 0x01, gear_code, 0, 0, linear_v1, linear_v2])

        try:
            # Standard CAN Frames (0x502 & 0x504)
            frame_502 = Frame(id_=0x502, data=steer_payload)
            frame_504 = Frame(id_=0x504, data=speed_payload)
            self.ch.write(frame_502)
            self.ch.write(frame_504)

            spaced_steer = " ".join(f"{b:02X}" for b in steer_payload)
            spaced_speed = " ".join(f"{b:02X}" for b in speed_payload)

        except canlib.CanError as e:
            if getattr(e, 'status', None) == canlib.ErrorNumber.TXBUFOVRFL or getattr(e, 'param', None) == -13:
                pass
            else:
                print(f"[{self.name}] CAN Error sending frames: {e}")
        except Exception as e:
            print(f"[{self.name}] Unexpected error sending CAN frames: {e}")

    def degree_to_can_cmd(self, angle_deg: float) -> int:
        """
        Map degree (-28.0 to +28.0) to raw CAN command (-2000 to +2000).
        Left: -2000 (at -28 deg), Right: +2000 (at +28 deg)
        """
        clamped_deg = max(self.MIN_ANGLE_DEG, min(self.MAX_ANGLE_DEG, float(angle_deg)))
        cmd_val = int(round((clamped_deg / self.MAX_ANGLE_DEG) * self.MAX_CMD_VAL))
        return max(self.MIN_CMD_VAL, min(self.MAX_CMD_VAL, cmd_val))

    def can_cmd_to_degree(self, cmd_val: int) -> float:
        """Map raw CAN command (-2000 to +2000) back to degree (-28.0 to +28.0)."""
        clamped_cmd = max(self.MIN_CMD_VAL, min(self.MAX_CMD_VAL, int(cmd_val)))
        return (clamped_cmd / float(self.MAX_CMD_VAL)) * self.MAX_ANGLE_DEG

    def set_steering_angle(self, angle_deg: float):
        """Update target steering angle in degrees."""
        clamped_angle = max(self.MIN_ANGLE_DEG, min(self.MAX_ANGLE_DEG, float(angle_deg)))
        self.steer_angle = clamped_angle

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
        return {
            "name": self.name,
            "channel": self.channel,
            "connected": self.is_connected,
            "speed": self.speed,
            "steer_angle": self.steer_angle,
            "can_cmd_val": self.degree_to_can_cmd(self.steer_angle),
            "latitude": self.lat,
            "longitude": self.lon,
            "gear": self.gear,
            "drive_mode": self.drive_mode,
            "heading": self.simulated_heading,
            "parsed_can_status": self.parsed_can_status
        }
