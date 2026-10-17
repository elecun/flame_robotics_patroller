"""Fluke SV600 acoustic imager REST/WebSocket integration.

The SV600 exposes a device hosted HTTP REST API and WebSocket streams. API
paths vary with firmware/API document revision, so paths are configurable.
Frames are retained as bytes and optionally written per frame to disk.
"""
import base64
import json
import os
import threading
import time
from datetime import datetime, timezone
from typing import Any, Callable, Optional
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from core.device.base import BaseDevice
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()

try:
    import websockets.sync.client
except ImportError:
    websockets = None
else:
    import websockets


class FlukeSV600(BaseDevice):
    """Control frequency bands and receive/save SV600 acoustic image frames."""
    def __init__(self, name="fluke_sv600", host="192.168.100.20", port=80,
                 username="admin", password="", frequency_ranges="20000-28000",
                 image_output_dir="datalog/fluke_sv600", stream_path="/api/v1/stream",
                 frequency_path="/api/v1/frequency-ranges", timeout=5.0,
                 enable=True, on_frame: Optional[Callable[[bytes, dict], None]]=None, **kwargs):
        super().__init__(name, enable=enable)
        self.base_url = f"http://{host}:{int(port)}"
        self.stream_url = f"ws://{host}:{int(port)}{stream_path}"
        self.username, self.password = username, password
        self.frequency_path = frequency_path
        self.frequency_ranges = self._parse_ranges(frequency_ranges)
        self.image_output_dir = image_output_dir
        self.timeout = float(timeout)
        self.on_frame = on_frame
        self.last_frame: Optional[bytes] = None
        self.last_frame_metadata: dict = {}
        self.frame_count = 0
        self._running = False
        self._thread = None

    @staticmethod
    def _parse_ranges(value):
        if isinstance(value, str):
            value = [part.strip() for part in value.split(",") if part.strip()]
        ranges = []
        for item in value:
            low, high = (item.split("-", 1) if isinstance(item, str) else item)
            low, high = int(low), int(high)
            if low < 2000 or high > 52000 or low >= high:
                raise ValueError("SV600 frequencies must be increasing and within 2000-52000 Hz")
            ranges.append([low, high])
        if sum(hi-lo for lo, hi in ranges) > 8000:
            raise ValueError("SV600 selected frequency bandwidth cannot exceed 8000 Hz")
        return ranges

    def _request(self, path, method="GET", payload=None):
        data = json.dumps(payload).encode() if payload is not None else None
        headers = {"Accept": "application/json"}
        if data is not None:
            headers["Content-Type"] = "application/json"
        req = Request(self.base_url + path, data=data, headers=headers, method=method)
        # Basic auth is supported by firmware versions configured for HTTP auth.
        if self.username:
            import urllib.request
            manager = urllib.request.HTTPPasswordMgrWithDefaultRealm()
            manager.add_password(None, self.base_url, self.username, self.password)
            opener = urllib.request.build_opener(urllib.request.HTTPBasicAuthHandler(manager))
            response = opener.open(req, timeout=self.timeout)
        else:
            response = urlopen(req, timeout=self.timeout)
        with response:
            body = response.read()
        if not body:
            return None
        try:
            return json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return body

    def set_frequency_ranges(self, ranges):
        """Set selected Hz bands through the configurable SV600 REST endpoint."""
        parsed = self._parse_ranges(ranges)
        result = self._request(self.frequency_path, "PUT", {"frequencyRanges": parsed})
        self.frequency_ranges = parsed
        return result

    def receive_frame(self, frame, metadata=None, save=True):
        """Accept an image frame (bytes or JSON with image/data base64) and save it."""
        metadata = dict(metadata or {})
        if isinstance(frame, dict):
            metadata = {**frame, **metadata}
            encoded = frame.get("image", frame.get("data", ""))
            frame = base64.b64decode(encoded) if isinstance(encoded, str) else bytes(encoded)
        elif isinstance(frame, str):
            frame = base64.b64decode(frame)
        frame = bytes(frame)
        self.last_frame, self.last_frame_metadata = frame, metadata
        self.frame_count += 1
        if save:
            os.makedirs(self.image_output_dir, exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            ext = metadata.get("format", "jpg").lower().lstrip(".")
            path = os.path.join(self.image_output_dir, f"sv600_{stamp}_{self.frame_count:06d}.{ext}")
            with open(path, "wb") as image_file:
                image_file.write(frame)
            metadata["file"] = path
            self.last_frame_metadata = metadata
        if self.on_frame:
            self.on_frame(frame, metadata)
        return frame

    def _stream_loop(self):
        while self._running:
            try:
                headers = {"Authorization": "Basic " + base64.b64encode(f"{self.username}:{self.password}".encode()).decode()} if self.username else None
                with websockets.sync.client.connect(self.stream_url, additional_headers=headers, open_timeout=self.timeout) as ws:
                    while self._running:
                        message = ws.recv(timeout=self.timeout)
                        if isinstance(message, bytes):
                            self.receive_frame(message)
                        else:
                            payload = json.loads(message)
                            self.receive_frame(payload)
            except Exception as exc:
                if self._running:
                    logger.warning(f"[{self.name}] Stream disconnected; retrying: {exc}")
                    time.sleep(1)

    def connect(self):
        if not self.enable:
            return False
        if websockets is None:
            logger.error(f"[{self.name}] websockets package is required for live image streaming")
            return False
        self._running = True
        self._thread = threading.Thread(target=self._stream_loop, daemon=True, name=f"{self.name}_stream")
        self._thread.start()
        self.is_connected = True
        return True

    def disconnect(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None
        self.is_connected = False
        return True

    def get_status(self):
        return {"connected": self.is_connected, "frame_count": self.frame_count,
                "frequency_ranges": self.frequency_ranges,
                "last_frame_metadata": self.last_frame_metadata}
