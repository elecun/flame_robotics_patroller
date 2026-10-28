"""Record data at the device receive points into a timestamped session."""
import csv
import json
import os
import queue
import socket
import shutil
import struct
import threading
import time
from datetime import datetime
from typing import Optional, Any

from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class DataLogger:
    def __init__(self, robot: Any, base_dir: Optional[str] = None):
        self.robot = robot
        cfg = getattr(robot, 'config', None)
        section = cfg['DATA_LOGGER'] if cfg and cfg.has_section('DATA_LOGGER') else None
        self.base_dir = base_dir or (section.get('base_dir', '').strip() if section else '') or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'datalog')
        self._config = section
        self._lock = threading.Lock()
        self._lifecycle_lock = threading.RLock()
        self._session_token = None
        self._files = []
        self._hooks = []
        self._listeners = []
        self._workers = []
        self._session_dir = None
        self._start_time = None
        self.is_recording = False

    @property
    def session_dir(self):
        return self._session_dir

    @property
    def recording_duration(self):
        return time.time() - self._start_time if self.is_recording else 0.0

    def _enabled(self, key):
        return self._config is None or self._config.getboolean('enable_' + key, fallback=True)

    def _open(self, name, mode='w', **kwargs):
        f = open(os.path.join(self._session_dir, name), mode, **kwargs)
        self._files.append(f)
        return f

    def _make_recorder(self, dev, callback):
        # Each device owns its queue and writer so slow storage for one device
        # cannot block another device's receiver or writer.
        pending = queue.SimpleQueue()
        token = self._session_token
        def worker():
            while True:
                item = pending.get()
                if item is None:
                    return
                timestamp, args = item
                try:
                    callback(timestamp, *args)
                except Exception as exc:
                    logger.error(f'[DataLogger] {dev.name} write error: {exc}')
        thread = threading.Thread(target=worker, name=f'datalog-{dev.name}', daemon=True)
        thread.start()
        self._workers.append((pending, thread))
        def enqueue(*args):
            timestamp = time.time()
            with self._lock:
                if self.is_recording and self._session_token is token:
                    pending.put((timestamp, args))
        return enqueue

    def _attach(self, dev, attr, callback):
        recorder = self._make_recorder(dev, callback)
        setattr(dev, attr, recorder)
        self._hooks.append((dev, attr, recorder))

    def _pcap(self, dev, name):
        f = self._open(name + '.pcap', 'wb')
        f.write(struct.pack('<IHHiIII', 0xa1b2c3d4, 2, 4, 0, 0, 65535, 1))
        writer = _VLP16RecordingSocket(None, f, getattr(dev, 'ip', None), dev.port)
        writer._dst_ip = getattr(dev, 'bind_ip', '0.0.0.0')
        self._attach(dev, 'data_logger_packet', lambda ts, data, addr: writer._write_pcap_record(data, addr, ts))

    def _images(self, dev, name):
        directory = os.path.join(self._session_dir, name)
        os.makedirs(directory, exist_ok=True)
        index = [0]
        metadata_file = self._open(name + '_metadata.jsonl', 'w', encoding='utf-8')
        def save(timestamp, frame, metadata):
            index[0] += 1
            stamp = int(timestamp * 1e9)
            filename = f'{stamp}_{index[0]:08d}.jpg'
            with open(os.path.join(directory, filename), 'wb') as out:
                out.write(frame)
            metadata_file.write(json.dumps({'received_timestamp': stamp / 1e9, 'file': filename, 'metadata': metadata}, default=str) + '\n')
        self._attach(dev, 'data_logger_frame', save)

    def start_recording(self):
        with self._lifecycle_lock:
            if self.is_recording:
                return
            os.makedirs(self.base_dir, exist_ok=True)
            self._session_dir = os.path.join(self.base_dir, datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            os.makedirs(self._session_dir)
            self._start_time = time.time()
            with self._lock:
                self._session_token = object()
                self.is_recording = True
            devices = getattr(self.robot, 'devices', {})
            try:
                for name, dev in devices.items():
                    if not getattr(dev, 'enable', True):
                        continue
                    if name in ('vlp-16', 'ouster-sr-128') and self._enabled('vlp16' if name == 'vlp-16' else 'ouster'):
                        self._pcap(dev, name)
                        if name == 'ouster-sr-128':
                            info = getattr(dev, '_sensor_info', None)
                            if info is not None:
                                with open(os.path.join(self._session_dir, name + '.json'), 'w') as out:
                                    out.write(info.to_json_string())
                            else:
                                metadata = getattr(dev, 'metadata_path', '')
                                if metadata and not os.path.isabs(metadata):
                                    metadata = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), metadata)
                                if metadata and os.path.isfile(metadata):
                                    shutil.copyfile(metadata, os.path.join(self._session_dir, name + '.json'))
                                else:
                                    logger.warning('[DataLogger] Ouster metadata unavailable at recording start')
                    elif name in ('basler_gige_camera', 'fluke_sv600') and self._enabled('camera' if name == 'basler_gige_camera' else 'fluke'):
                        self._images(dev, name)
                    elif name in ('synerex_syn2000', 'synerex_syn3100', 'synerex_rtk') and self._enabled('rtk'):
                        f = self._open(name + '.nmea', 'w', encoding='utf-8')
                        def save_nmea(timestamp, line, out=f):
                            out.write(f'{timestamp:.9f} {line}\n')
                        listener = self._make_recorder(dev, save_nmea)
                        dev.add_nmea_listener(listener)
                        self._listeners.append((dev, listener))
                    elif name == 'xsens_mti_630r' and self._enabled('xsens'):
                        f = self._open(name + '.jsonl', 'w', encoding='utf-8')
                        def save_xsens(timestamp, data, can_id, out=f):
                            out.write(json.dumps({'timestamp': timestamp, 'can_id': can_id, 'data_hex': data.hex()}) + '\n')
                        self._attach(dev, 'data_logger_raw', save_xsens)
                    elif name == 'mobile_drive_s1' and self._enabled('can'):
                        f = self._open('mobile_can.csv', 'w', newline='')
                        writer = csv.writer(f)
                        writer.writerow(['timestamp', 'id', 'data_hex', 'speed_kmh', 'steer_angle_deg'])
                        def save_can(timestamp, frame, out=writer, device=dev):
                            parsed = device.parser.parse(frame.id, frame.data) if frame.id == 0x304 else {}
                            out.writerow([f'{timestamp:.9f}', f'0x{frame.id:X}', bytes(frame.data).hex(),
                                          parsed.get('vehicle_velocity_val', ''), parsed.get('vehicle_steer_angle_val', '')])
                        self._attach(dev, 'frame_recorder', save_can)
                logger.info(f'[DataLogger] Recording started -> {self._session_dir}')
            except Exception:
                self.stop_recording()
                raise

    def stop_recording(self):
        with self._lifecycle_lock:
            with self._lock:
                if not self.is_recording:
                    return
                self.is_recording = False
                self._session_token = None
            for dev, attr, callback in self._hooks:
                if getattr(dev, attr, None) is callback:
                    setattr(dev, attr, None)
            self._hooks.clear()
            for dev, listener in self._listeners:
                dev.remove_nmea_listener(listener)
            self._listeners.clear()
            for pending, thread in self._workers:
                pending.put(None)
            for pending, thread in self._workers:
                thread.join()
            self._workers.clear()
            for f in self._files:
                f.close()
            self._files.clear()
            logger.info(f'[DataLogger] Recording stopped -> {self._session_dir}')


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

    def _write_pcap_record(self, payload: bytes, addr, timestamp=None):
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
        dst_ip_bytes = socket.inet_aton(getattr(self, "_dst_ip", "0.0.0.0"))
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

        # Compute the IPv4 header checksum.
        words = struct.unpack("!10H", ip_header)
        checksum = sum(words)
        checksum = (checksum & 0xffff) + (checksum >> 16)
        checksum = (checksum & 0xffff) + (checksum >> 16)
        ip_header = ip_header[:10] + struct.pack("!H", (~checksum) & 0xffff) + ip_header[12:]

        # Build Ethernet header (14 bytes)
        eth_header = b'\x00' * 6 + b'\x00' * 6 + struct.pack("!H", 0x0800)  # IPv4

        packet = eth_header + ip_header + udp_header + payload

        # Pcap record header: ts_sec, ts_usec, incl_len, orig_len
        now = time.time() if timestamp is None else timestamp
        ts_sec = int(now)
        ts_usec = int((now - ts_sec) * 1_000_000)
        record_header = struct.pack("<IIII", ts_sec, ts_usec, len(packet), len(packet))

        with self._lock:
            self._pcap_file.write(record_header + packet)

    # Delegate all other socket methods to real socket
    def __getattr__(self, name):
        return getattr(self._real_sock, name)


