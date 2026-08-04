"""
APROS Data Logger Module (core/data_logger.py)
Records all incoming sensor data to timestamped directories under APROS/datalog.
- VLP-16: raw UDP packet pcap file
- Ouster-SR-128: pcap + sensor metadata JSON (via ouster-sdk)
- Basler GigE Camera: JPEG images with millisecond timestamps
- Mobile Drive S1: CAN bus log CSV
- Baumer Incline: incline data CSV
"""

import os
import time
import struct
import socket
import threading
import csv
import json
import pickle
import numpy as np
from datetime import datetime
from typing import Optional, Dict, Any

from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class DataLogger:
    """
    Central data logger that records all incoming robot sensor data
    to a timestamped session directory under APROS/datalog/.
    """

    def __init__(self, robot: Any, base_dir: Optional[str] = None):
        self.robot = robot
        self.is_recording = False
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._session_dir: Optional[str] = None
        self._start_time: Optional[float] = None

        # Configurable Parameters from apros.cfg [DATA_LOGGER] section
        cfg_base_dir = ""
        self.record_interval_sec = 0.05  # Default 50ms (20Hz)
        self.enable_camera_log = True
        self.enable_vlp16_log = True
        self.enable_ouster_log = True
        self.enable_can_log = True
        self.enable_incline_log = True
        self.enable_rtk_log = True

        if hasattr(self.robot, "config") and self.robot.config and self.robot.config.has_section("DATA_LOGGER"):
            dl_cfg = self.robot.config["DATA_LOGGER"]
            cfg_base_dir = dl_cfg.get("base_dir", "").strip()
            interval_ms = float(dl_cfg.get("record_interval_ms", 50))
            self.record_interval_sec = max(0.001, interval_ms / 1000.0)

            self.enable_camera_log = dl_cfg.getboolean("enable_camera", True)
            self.enable_vlp16_log = dl_cfg.getboolean("enable_vlp16", True)
            self.enable_ouster_log = dl_cfg.getboolean("enable_ouster", True)
            self.enable_can_log = dl_cfg.getboolean("enable_can", True)
            self.enable_incline_log = dl_cfg.getboolean("enable_incline", True)
            self.enable_rtk_log = dl_cfg.getboolean("enable_rtk", True)

        # Base directory for data logs
        if base_dir:
            self.base_dir = base_dir
        elif cfg_base_dir:
            self.base_dir = cfg_base_dir
        else:
            # Default: APROS/datalog relative to project root
            apros_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            self.base_dir = os.path.join(apros_root, "datalog")

        # Per-device file handles and state
        self._cam_dir: Optional[str] = None
        self._vlp16_pcap_file = None
        self._vlp16_pcap_path: Optional[str] = None
        self._vlp16_orig_sock: Optional[socket.socket] = None
        self._ouster_pcap_path: Optional[str] = None
        self._ouster_json_path: Optional[str] = None
        self._ouster_recording_source = None

        self._can_csv_file = None
        self._can_csv_writer = None
        self._incline_csv_file = None
        self._incline_csv_writer = None

        # Tracking last recorded timestamps to avoid duplicates
        self._last_cam_ts = 0.0
        self._last_ouster_pts_id = 0

    def start_recording(self):
        """Start recording all sensor data to a new session directory."""
        if self.is_recording:
            logger.warning("[DataLogger] Already recording.")
            return

        # Create session directory with timestamp
        now = datetime.now()
        session_name = now.strftime("%Y%m%d_%H%M%S")
        self._session_dir = os.path.join(self.base_dir, session_name)
        os.makedirs(self._session_dir, exist_ok=True)

        self._start_time = time.time()
        self.is_recording = True
        self._running = True

        # Initialize sub-directories and file handles based on configuration flags
        if self.enable_camera_log:
            self._init_camera_logging()
        if self.enable_vlp16_log:
            self._init_vlp16_logging()
        if self.enable_ouster_log:
            self._init_ouster_logging()
        if self.enable_can_log:
            self._init_can_logging()
        if self.enable_incline_log:
            self._init_incline_logging()
        if self.enable_rtk_log:
            self._init_rtk_logging()

        # Start background recording thread
        self._thread = threading.Thread(target=self._recording_loop, daemon=True)
        self._thread.start()

        logger.info(f"[DataLogger] Recording started -> {self._session_dir}")

    def stop_recording(self):
        """Stop recording and finalize all file handles."""
        if not self.is_recording:
            return

        self._running = False
        self.is_recording = False

        if self._thread:
            self._thread.join(timeout=3.0)
            self._thread = None

        # Close file handles
        self._close_vlp16_logging()
        self._close_can_logging()
        self._close_incline_logging()
        self._close_rtk_logging()
        self._close_ouster_logging()

        elapsed = time.time() - self._start_time if self._start_time else 0.0
        logger.info(f"[DataLogger] Recording stopped. Duration: {elapsed:.1f}s, Dir: {self._session_dir}")

    @property
    def session_dir(self) -> Optional[str]:
        return self._session_dir

    @property
    def recording_duration(self) -> float:
        if self._start_time and self.is_recording:
            return time.time() - self._start_time
        return 0.0

    # ──────────────────────────────────────────────
    # Camera Logging
    # ──────────────────────────────────────────────
    def _init_camera_logging(self):
        self._cam_dir = os.path.join(self._session_dir, "basler_gige_camera")
        os.makedirs(self._cam_dir, exist_ok=True)
        self._last_cam_ts = 0.0

    def _record_camera_frame(self):
        """Save current JPEG camera frame with millisecond timestamp filename."""
        if not self._cam_dir:
            return
        frame_bytes = getattr(self.robot, "last_camera_frame", None)
        cam_hdr = getattr(self.robot, "last_camera_header", None) or {}
        ts = cam_hdr.get("timestamp", 0.0)

        if frame_bytes and isinstance(frame_bytes, bytes) and len(frame_bytes) > 0 and ts != self._last_cam_ts:
            self._last_cam_ts = ts
            now = datetime.now()
            filename = now.strftime("%Y%m%d_%H%M%S_") + f"{now.microsecond // 1000:03d}.jpg"
            filepath = os.path.join(self._cam_dir, filename)
            try:
                with open(filepath, "wb") as f:
                    f.write(frame_bytes)
            except Exception as e:
                logger.error(f"[DataLogger] Camera frame write error: {e}")

    # ──────────────────────────────────────────────
    # VLP-16 Logging (raw UDP pcap)
    # ──────────────────────────────────────────────
    def _init_vlp16_logging(self):
        """Initialize VLP-16 pcap recording by wrapping its UDP socket to intercept raw packets."""
        vlp_dev = self.robot.devices.get("vlp-16") if hasattr(self.robot, "devices") else None
        if vlp_dev is None or not vlp_dev.is_connected or vlp_dev.sock is None:
            self._vlp16_pcap_file = None
            return

        self._vlp16_pcap_path = os.path.join(self._session_dir, "vlp-16.pcap")
        try:
            self._vlp16_pcap_file = open(self._vlp16_pcap_path, "wb")
            # Write pcap global header (magic, version 2.4, snaplen 65535, linktype Ethernet=1)
            self._vlp16_pcap_file.write(struct.pack(
                "<IHHiIII",
                0xa1b2c3d4,  # magic number
                2, 4,        # version major, minor
                0,           # thiszone (GMT)
                0,           # sigfigs
                65535,       # snaplen
                1            # network (LINKTYPE_ETHERNET)
            ))

            # Replace VLP-16 socket with a recording wrapper
            self._vlp16_orig_sock = vlp_dev.sock
            vlp_dev.sock = _VLP16RecordingSocket(self._vlp16_orig_sock, self._vlp16_pcap_file, vlp_dev.ip, vlp_dev.port)
            logger.info(f"[DataLogger] VLP-16 pcap recording started: {self._vlp16_pcap_path}")
        except Exception as e:
            logger.error(f"[DataLogger] VLP-16 pcap init error: {e}")
            self._vlp16_pcap_file = None

    def _close_vlp16_logging(self):
        """Restore original VLP-16 socket and close pcap file."""
        vlp_dev = self.robot.devices.get("vlp-16") if hasattr(self.robot, "devices") else None
        if vlp_dev is not None and self._vlp16_orig_sock is not None:
            vlp_dev.sock = self._vlp16_orig_sock
            self._vlp16_orig_sock = None
        if self._vlp16_pcap_file:
            try:
                self._vlp16_pcap_file.close()
            except Exception:
                pass
            self._vlp16_pcap_file = None
            logger.info(f"[DataLogger] VLP-16 pcap recording finalized: {self._vlp16_pcap_path}")

    # ──────────────────────────────────────────────
    # Ouster-SR-128 Logging (pcap + json)
    # ──────────────────────────────────────────────
    # ──────────────────────────────────────────────
    # Ouster-SR-128 Logging (pcap + json)
    # ──────────────────────────────────────────────
    def _init_ouster_logging(self):
        """Initialize Ouster pcap recording using live sensor metadata JSON and official ouster-sdk pcap binding."""
        ouster_dev = self.robot.devices.get("ouster-sr-128") if hasattr(self.robot, "devices") else None
        if ouster_dev is None or not ouster_dev.is_connected:
            self._ouster_pcap_handle = None
            return

        try:
            pcap_prefix = os.path.join(self._session_dir, "ouster-sr-128")

            # 1. Save LIVE sensor metadata JSON from connected sensor_info
            sensor_info = getattr(ouster_dev, "_sensor_info", None)
            self._ouster_json_path = pcap_prefix + ".json"
            if sensor_info is not None:
                json_str = sensor_info.to_json_string()
                with open(self._ouster_json_path, "w") as f:
                    f.write(json_str)
                logger.info(f"[DataLogger] Live Ouster metadata JSON saved: {self._ouster_json_path}")
            else:
                meta_template_path = os.path.join(self.base_dir, "meta", "OS-0-128-SR.json")
                if os.path.exists(meta_template_path):
                    import shutil
                    shutil.copyfile(meta_template_path, self._ouster_json_path)
                    logger.info(f"[DataLogger] Ouster metadata JSON copied from fallback template: {self._ouster_json_path}")

            # 2. Initialize official ouster-sdk C++ pcap record handle
            import ouster.sdk._bindings.pcap as pcap_b
            self._ouster_pcap_path = pcap_prefix + ".pcap"
            self._ouster_pcap_handle = pcap_b.record_initialize(self._ouster_pcap_path, 65535, False)
            self._ouster_pcap_lock = threading.Lock()

            # 3. Register callback on OusterSR128 device to record raw packets
            import ouster.sdk._bindings.client as cl

            def _record_raw_packet(pkt):
                if self._ouster_pcap_handle is None or pkt is None:
                    return
                try:
                    now_ts = time.time()
                    buf = getattr(pkt, "buf", None)
                    if buf is not None:
                        port = ouster_dev.port if isinstance(pkt, cl.LidarPacket) else (ouster_dev.port + 1)
                        with self._ouster_pcap_lock:
                            pcap_b.record_packet(
                                self._ouster_pcap_handle,
                                ouster_dev.ip, "192.168.100.2",
                                port, port,
                                buf, now_ts
                            )
                except Exception as pe:
                    logger.error(f"[DataLogger] Error recording raw Ouster packet: {pe}")

            ouster_dev.packet_recorder = _record_raw_packet
            logger.info(f"[DataLogger] Ouster pcap recording started via ouster-sdk C++ recorder: {self._ouster_pcap_path}")
        except Exception as e:
            logger.error(f"[DataLogger] Ouster recording init error: {e}")
            self._ouster_pcap_handle = None

    def _close_ouster_logging(self):
        """Finalize Ouster pcap recording."""
        ouster_dev = self.robot.devices.get("ouster-sr-128") if hasattr(self.robot, "devices") else None
        if ouster_dev is not None:
            ouster_dev.packet_recorder = None
        if hasattr(self, "_ouster_pcap_handle") and self._ouster_pcap_handle is not None:
            try:
                import ouster.sdk._bindings.pcap as pcap_b
                pcap_b.record_uninitialize(self._ouster_pcap_handle)
            except Exception as e:
                logger.error(f"[DataLogger] Ouster pcap finalize error: {e}")
            self._ouster_pcap_handle = None
            logger.info(f"[DataLogger] Ouster pcap recording finalized: {self._ouster_pcap_path}")

    # ──────────────────────────────────────────────
    # ──────────────────────────────────────────────
    # CAN Bus (Mobile Drive S1) Logging
    # ──────────────────────────────────────────────
    def _init_can_logging(self):
        drive_dev = self.robot.devices.get("mobile_drive_s1") if hasattr(self.robot, "devices") else None
        if drive_dev is None:
            return

        # 1. Status CSV logging
        can_csv_path = os.path.join(self._session_dir, "mobile_drive_s1.csv")
        try:
            self._can_csv_file = open(can_csv_path, "w", newline="")
            self._can_csv_writer = csv.writer(self._can_csv_file)
            self._can_csv_writer.writerow([
                "timestamp", "speed_kmh", "steer_angle_deg", "gear",
                "drive_mode", "can_cmd_val"
            ])
        except Exception as e:
            logger.error(f"[DataLogger] CAN CSV init error: {e}")

        # 2. Raw 0x301, 0x303 CAN Msg ID txt logging (mobile_can.txt)
        can_txt_path = os.path.join(self._session_dir, "mobile_can.txt")
        try:
            self._mobile_can_txt_file = open(can_txt_path, "a", encoding="utf-8")
            self._mobile_can_txt_lock = threading.Lock()

            def _record_can_frame(frame):
                if not self._mobile_can_txt_file or frame is None:
                    return
                if frame.id in (0x301, 0x303):
                    timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    hex_str = " ".join(f"{b:02X}" for b in frame.data)
                    log_line = f"[{timestamp_str}] ID: 0x{frame.id:03X} | Data: {hex_str}\n"
                    with self._mobile_can_txt_lock:
                        self._mobile_can_txt_file.write(log_line)
                        self._mobile_can_txt_file.flush()

            drive_dev.frame_recorder = _record_can_frame
            logger.info(f"[DataLogger] Raw CAN 0x301/0x303 logging started: {can_txt_path}")
        except Exception as e:
            logger.error(f"[DataLogger] mobile_can.txt init error: {e}")

    def _record_can_status(self):
        if not self._can_csv_writer:
            return
        drive_dev = self.robot.devices.get("mobile_drive_s1") if hasattr(self.robot, "devices") else None
        if drive_dev is None or not drive_dev.is_connected:
            return
        try:
            status = drive_dev.get_status()
            self._can_csv_writer.writerow([
                time.time(),
                status.get("speed", 0.0),
                status.get("steer_angle", 0.0),
                status.get("gear", "P"),
                status.get("drive_mode", "Manual"),
                status.get("can_cmd_val", 0)
            ])
        except Exception:
            pass

    def _close_can_logging(self):
        drive_dev = self.robot.devices.get("mobile_drive_s1") if hasattr(self.robot, "devices") else None
        if drive_dev is not None:
            drive_dev.frame_recorder = None
        if self._can_csv_file:
            try:
                self._can_csv_file.close()
            except Exception:
                pass
            self._can_csv_file = None
            self._can_csv_writer = None
        if hasattr(self, "_mobile_can_txt_file") and self._mobile_can_txt_file:
            try:
                self._mobile_can_txt_file.close()
            except Exception:
                pass
            self._mobile_can_txt_file = None

    # ──────────────────────────────────────────────
    # Synerex RTK Logging (CSV format)
    # ──────────────────────────────────────────────
    def _init_rtk_logging(self):
        rtk_dev = self.robot.devices.get("synerex_rtk") if hasattr(self.robot, "devices") else None
        if rtk_dev is None:
            return
        csv_path = os.path.join(self._session_dir, "synerex_rtk.csv")
        try:
            self._rtk_csv_file = open(csv_path, "w", newline="")
            self._rtk_csv_writer = csv.writer(self._rtk_csv_file)
            self._rtk_csv_writer.writerow([
                "timestamp", "iso_time", "latitude", "longitude", "altitude",
                "heading", "status", "satellites"
            ])
            logger.info(f"[DataLogger] Synerex RTK CSV logging started: {csv_path}")
        except Exception as e:
            logger.error(f"[DataLogger] Synerex RTK CSV init error: {e}")

    def _record_rtk_status(self):
        if not hasattr(self, "_rtk_csv_writer") or not self._rtk_csv_writer:
            return
        rtk_dev = self.robot.devices.get("synerex_rtk") if hasattr(self.robot, "devices") else None
        if rtk_dev is None or not rtk_dev.is_connected:
            return
        try:
            status = rtk_dev.get_status()
            now_ts = time.time()
            iso_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            self._rtk_csv_writer.writerow([
                now_ts,
                iso_str,
                status.get("latitude", 0.0),
                status.get("longitude", 0.0),
                status.get("altitude", 0.0),
                status.get("heading", 0.0),
                status.get("status", "FIX_RTK"),
                status.get("satellites", 0)
            ])
            self._rtk_csv_file.flush()
        except Exception as e:
            pass

    def _close_rtk_logging(self):
        if hasattr(self, "_rtk_csv_file") and self._rtk_csv_file:
            try:
                self._rtk_csv_file.close()
            except Exception:
                pass
            self._rtk_csv_file = None
            self._rtk_csv_writer = None

    # ──────────────────────────────────────────────
    # Baumer Incline Logging
    # ──────────────────────────────────────────────
    def _init_incline_logging(self):
        incline_dev = self.robot.devices.get("baumer_incline") if hasattr(self.robot, "devices") else None
        if incline_dev is None:
            return
        csv_path = os.path.join(self._session_dir, "baumer_incline.csv")
        try:
            self._incline_csv_file = open(csv_path, "w", newline="")
            self._incline_csv_writer = csv.writer(self._incline_csv_file)
            self._incline_csv_writer.writerow(["timestamp", "tilt_x", "tilt_z"])
        except Exception as e:
            logger.error(f"[DataLogger] Incline CSV init error: {e}")

    def _record_incline_status(self):
        if not self._incline_csv_writer:
            return
        incline_dev = self.robot.devices.get("baumer_incline") if hasattr(self.robot, "devices") else None
        if incline_dev is None or not incline_dev.is_connected:
            return
        try:
            status = incline_dev.get_status()
            self._incline_csv_writer.writerow([
                time.time(),
                status.get("tilt_x", 0.0),
                status.get("tilt_z", 0.0)
            ])
        except Exception:
            pass

    def _close_incline_logging(self):
        if self._incline_csv_file:
            try:
                self._incline_csv_file.close()
            except Exception:
                pass
            self._incline_csv_file = None
            self._incline_csv_writer = None

    # ──────────────────────────────────────────────
    # Main Recording Loop
    # ──────────────────────────────────────────────
    def _recording_loop(self):
        """Background thread that periodically records all sensor data."""
        logger.info("[DataLogger] Recording loop started.")
        while self._running:
            try:
                self._record_camera_frame()
                # VLP-16 pcap is recorded inline via socket wrapper
                self._record_can_status()
                self._record_incline_status()
                self._record_rtk_status()
                # Ouster pcap is recorded in its own dedicated thread
                time.sleep(self.record_interval_sec)
            except Exception as e:
                logger.error(f"[DataLogger] Recording loop error: {e}")
                time.sleep(0.1)
        logger.info("[DataLogger] Recording loop stopped.")


class _VLP16RecordingSocket:
    """
    Transparent socket wrapper that intercepts recvfrom() calls on VLP-16's UDP socket
    and writes each raw packet as a pcap record (with Ethernet + IP + UDP headers).
    """

    def __init__(self, real_sock: socket.socket, pcap_file, src_ip: str, dst_port: int):
        self._real_sock = real_sock
        self._pcap_file = pcap_file
        self._src_ip = src_ip
        self._dst_port = dst_port
        self._lock = threading.Lock()

    def recvfrom(self, bufsize: int):
        """Receive from real socket and record raw UDP payload into pcap."""
        data, addr = self._real_sock.recvfrom(bufsize)
        try:
            self._write_pcap_record(data, addr)
        except Exception:
            pass
        return data, addr

    def _write_pcap_record(self, payload: bytes, addr):
        """Write a single pcap packet record with Ethernet + IPv4 + UDP encapsulation."""
        src_ip_str = addr[0] if addr and len(addr) > 0 else self._src_ip
        src_port = addr[1] if addr and len(addr) > 1 else self._dst_port
        dst_port = self._dst_port

        # Build UDP header (8 bytes)
        udp_len = 8 + len(payload)
        udp_header = struct.pack("!HHHH", src_port, dst_port, udp_len, 0)  # checksum=0

        # Build IPv4 header (20 bytes, no options)
        ip_total_len = 20 + udp_len
        src_ip_bytes = socket.inet_aton(src_ip_str if src_ip_str else "192.168.100.12")
        dst_ip_bytes = socket.inet_aton("192.168.100.2")  # Match udp_dest in metadata JSON
        ip_header = struct.pack("!BBHHHBBH4s4s",
            0x45,        # version=4, IHL=5
            0,           # DSCP/ECN
            ip_total_len,
            0, 0,        # identification, flags/fragment
            64,          # TTL
            17,          # protocol (UDP)
            0,           # checksum (0 = not computed)
            src_ip_bytes,
            dst_ip_bytes
        )

        # Build Ethernet header (14 bytes)
        eth_header = b'\x00' * 6 + b'\x00' * 6 + struct.pack("!H", 0x0800)  # IPv4

        packet = eth_header + ip_header + udp_header + payload

        # Pcap record header: ts_sec, ts_usec, incl_len, orig_len
        now = time.time()
        ts_sec = int(now)
        ts_usec = int((now - ts_sec) * 1_000_000)
        record_header = struct.pack("<IIII", ts_sec, ts_usec, len(packet), len(packet))

        with self._lock:
            self._pcap_file.write(record_header + packet)

    # Delegate all other socket methods to real socket
    def __getattr__(self, name):
        return getattr(self._real_sock, name)


class _OusterRecordingSocket(_VLP16RecordingSocket):
    """Transparent socket wrapper that intercepts Ouster UDP socket recvfrom calls to write pcap records."""
    pass

