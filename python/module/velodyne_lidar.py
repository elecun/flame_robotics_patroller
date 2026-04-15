"""
Velodyne VLP-16 Lidar Module
@author Byunghun Hwang<bh.hwang@iae.re.kr>
"""

try:
    from PyQt6.QtCore import QThread, pyqtSignal
except ImportError:
    print("PyQt6 is required to run this module.")
    class QThread:
        def __init__(self): pass
    class pyqtSignal:
        def __init__(self, *args, **kwargs): pass
        def emit(self, *args, **kwargs): pass

import socket
import struct
import math
import threading
import queue
import numpy as np
from contextlib import closing
from typing import Optional, List
from scapy.all import wrpcap, Ether, IP, UDP


# --------------------------------------------------------------------------
# VLP-16 Protocol constants
# --------------------------------------------------------------------------
VLP16_PACKET_SIZE    = 1206   # Data packet size in bytes
VLP16_BLOCKS_PER_PKT = 12     # Firing blocks per packet
VLP16_CHANNELS       = 16     # Laser channels (beams)
VLP16_BLOCK_SIZE     = 100    # Bytes per block
VLP16_HEADER_SIZE    = 42     # Bytes before data blocks
VLP16_DATA_OFFSET    = 0

# Vertical angles for the 16 laser beams (degrees), ordered by laser_id
VLP16_VERTICAL_ANGLES = [
    -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0,
     -7.0, 9.0,  -5.0, 11.0, -3.0, 13.0, -1.0, 15.0
]
VLP16_V_ANGLES_RAD = [math.radians(a) for a in VLP16_VERTICAL_ANGLES]

# One full revolution = 36000 (azimuth in 0.01-degree units)
FULL_REVOLUTION = 36000


def parse_vlp16_packet(data: bytes,
                        range_min: float,
                        range_max: float) -> Optional[np.ndarray]:
    """
    Parse a single VLP-16 UDP packet into an Nx4 numpy array [x, y, z, intensity].

    Returns None if the packet is not a valid VLP-16 data packet.
    """
    if len(data) < VLP16_PACKET_SIZE:
        return None

    points: List[List[float]] = []

    # Parse 12 firing blocks
    for block_idx in range(VLP16_BLOCKS_PER_PKT):
        offset = VLP16_DATA_OFFSET + block_idx * VLP16_BLOCK_SIZE

        # Block header: 0xFFEE flag + azimuth
        flag = struct.unpack_from('<H', data, offset)[0]
        if flag != 0xFFEE:
            continue

        azimuth_raw = struct.unpack_from('<H', data, offset + 2)[0]  # 0.01 deg units
        azimuth_rad = math.radians(azimuth_raw / 100.0)

        # 16 channels × (distance 2 bytes + intensity 1 byte)
        channel_offset = offset + 4
        for ch in range(VLP16_CHANNELS):
            ch_off = channel_offset + ch * 3
            dist_raw  = struct.unpack_from('<H', data, ch_off)[0]
            intensity = struct.unpack_from('B', data,  ch_off + 2)[0]

            distance_m = dist_raw * 0.002   # 2 mm per unit

            if distance_m < range_min or distance_m > range_max:
                continue

            v_angle = VLP16_V_ANGLES_RAD[ch]
            xy = distance_m * math.cos(v_angle)
            x  = xy * math.sin(azimuth_rad)
            y  = xy * math.cos(azimuth_rad)
            z  = distance_m * math.sin(v_angle)

            points.append([x, y, z, float(intensity)])

    if not points:
        return None

    return np.array(points, dtype=np.float32)


class component(QThread):
    """
    Velodyne VLP-16 receiver thread.

    * Listens for UDP packets on the configured port.
    * Assembles one full 360-degree scan per revolution.
    * Emits a dict payload via signal_updated once per revolution.

    Config keys (velodyne_lidar.cfg):
        device_ip           : Sensor IP address
        port                : UDP port (default 2368)
        rpm                 : Sensor rotation speed (affects scan assembly timeout)
        range_filter_min_m  : Minimum valid range (m)
        range_filter_max_m  : Maximum valid range (m)
        max_points_per_frame: Downsample cap (0 = disabled)
        point_size          : Visual point size (forwarded to renderer)
        colormap            : Colormap name (forwarded to renderer)
        background_color    : Background hex color (forwarded to renderer)
    """

    signal_updated = pyqtSignal(object)

    def __init__(self,
                 device_ip:            str   = "192.168.1.201",
                 port:                 int   = 2368,
                 rpm:                  int   = 600,
                 range_filter_min_m:   float = 0.5,
                 range_filter_max_m:   float = 100.0,
                 max_points_per_frame: int   = 30000,
                 point_size:           int   = 3,
                 colormap:             str   = "jet",
                 background_color:     str   = "#1a1a2e"):
        super().__init__()

        self.device_ip   = device_ip
        self.port        = int(port)
        self.rpm         = int(rpm)
        self.range_min   = float(range_filter_min_m)
        self.range_max   = float(range_filter_max_m)
        self.max_pts     = int(max_points_per_frame)

        # Rendering hints — read by tab_3dscan
        self.point_size       = int(point_size)
        self.colormap         = colormap
        self.background_color = background_color

        self.running      = False
        self._stop_event  = threading.Event()
        self._sock: Optional[socket.socket] = None

        self._recording   = False
        self._record_path: Optional[str] = None

    # ------------------------------------------------------------------
    # Thread main loop
    # ------------------------------------------------------------------
    def run(self):
        self.running = True
        self._stop_event.clear()
        print(f"[Velodyne] Connecting {self.device_ip}:{self.port} ...")

        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.bind(('', self.port))
            self._sock.settimeout(1.0)
            print("[Velodyne] Listening for packets ...")

            frame_points: List[np.ndarray] = []
            last_azimuth = -1

            while not self._stop_event.is_set():
                try:
                    data, addr = self._sock.recvfrom(2048)
                except socket.timeout:
                    continue
                except OSError:
                    # Socket closed externally
                    break

                if addr[0] != self.device_ip:
                    continue

                # Recording
                if self._recording and self._record_path:
                    pkt = (Ether() /
                           IP(src=addr[0], dst="255.255.255.255") /
                           UDP(sport=self.port, dport=self.port) / data)
                    wrpcap(self._record_path, pkt, append=True)

                pts = parse_vlp16_packet(data, self.range_min, self.range_max)
                if pts is None:
                    continue

                # Detect one full revolution by watching azimuth wrap-around
                if len(data) >= VLP16_DATA_OFFSET + 4:
                    azimuth_raw = struct.unpack_from('<H', data, VLP16_DATA_OFFSET + 2)[0]
                    if last_azimuth >= 0 and azimuth_raw < last_azimuth:
                        # Revolution complete — build and emit frame
                        self._emit_frame(frame_points)
                        frame_points = []
                    last_azimuth = azimuth_raw

                frame_points.append(pts)

        except Exception as e:
            print(f"[Velodyne] Error: {e}")
        finally:
            if self._sock:
                self._sock.close()
                self._sock = None
            self.running = False
            print("[Velodyne] Stream stopped.")

    # ------------------------------------------------------------------
    # Frame assembly → emit
    # ------------------------------------------------------------------
    def _emit_frame(self, frame_points: List[np.ndarray]):
        if not frame_points:
            return

        cloud = np.concatenate(frame_points, axis=0)  # (N, 4)  [x,y,z,intensity]

        # Downsample if cap set
        if self.max_pts > 0 and len(cloud) > self.max_pts:
            idx = np.random.choice(len(cloud), self.max_pts, replace=False)
            cloud = cloud[idx]

        payload = {
            "xyz":        cloud[:, :3],          # (N, 3) float32
            "intensity":  cloud[:, 3],            # (N,)   float32
            "point_size": self.point_size,
            "colormap":   self.colormap,
            "bg_color":   self.background_color,
        }
        self.signal_updated.emit(payload)

    # ------------------------------------------------------------------
    # Recording
    # ------------------------------------------------------------------
    def start_record(self, filepath: str):
        self._record_path = filepath
        self._recording   = True

    def stop_record(self):
        self._recording = False

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def stop(self):
        print("[Velodyne] Stopping ...")
        self._stop_event.set()
        self.running = False
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
        if not self.wait(3000):
            print("[Velodyne] Warning: thread did not terminate in time.")

    def __del__(self):
        self.stop()