"""Fluke SV600 acoustic imager REST integration.

Poll the device's SoundMap capture API for images containing the acoustic
overlay. Frames are retained as bytes and can optionally be saved to disk.
"""
import base64
import copy
import json
import os
import threading
import time
from datetime import datetime, timezone
from typing import Any, Callable, Optional
from urllib.request import Request, urlopen

from core.device.base import BaseDevice
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()

class FlukeSV600(BaseDevice):
    """Control frequency bands and receive/save SV600 acoustic image frames."""
    def __init__(self, name="fluke_sv600", host="192.168.100.20", port=80,
                 username="admin", password="", frequency_ranges="20000-28000",
                 image_output_dir="datalog/fluke_sv600", api_port=9011,
                 capture_interval_sec=1.0, save_frames=False,
                 frequency_path="/api/v1/frequency-ranges", timeout=5.0,
                 enable=True, on_frame: Optional[Callable[[bytes, dict], None]]=None, **kwargs):
        super().__init__(name, enable=enable)
        self.base_url = f"http://{host}:{int(port)}"
        self.api_url = f"http://{host}:{int(api_port)}"
        self.username, self.password = username, password
        self.frequency_path = frequency_path
        self.frequency_ranges = self._parse_ranges(frequency_ranges)
        self.active_frequency_ranges = None
        self.image_output_dir = image_output_dir
        self.timeout = float(timeout)
        self.capture_interval_sec = max(0.1, float(capture_interval_sec))
        self.save_frames = bool(save_frames)
        self.on_frame = on_frame
        self.last_frame: Optional[bytes] = None
        self.last_frame_metadata: dict = {}
        self.frame_count = 0
        self._running = False
        self._thread = None
        self._settings_lock = threading.Lock()

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
        if self.username:
            credentials = base64.b64encode(f"{self.username}:{self.password}".encode()).decode()
            headers["Authorization"] = f"Basic {credentials}"
        req = Request(self.api_url + path, data=data, headers=headers, method=method)
        with urlopen(req, timeout=self.timeout) as response:
            body = response.read()
        if not body:
            return None
        try:
            return json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return body

    def set_frequency_ranges(self, ranges):
        """Apply the requested bands to the active SoundMap on the SV600."""
        parsed = self._parse_ranges(ranges)
        with self._settings_lock:
            surfaces = self._request("/soundsurfaces")
            surface = next((item for item in surfaces if item.get("settings", {}).get("properties", {}).get("type") == "Data"), None)
            if surface is None:
                raise RuntimeError("No active SoundMap data surface is available")
            settings = copy.deepcopy(surface["settings"])
            settings["frequencyRanges"] = [{"from": low, "to": high} for low, high in parsed]
            self._request(f"/soundsurfaces/{surface['id']}", "PUT", settings)
            updated_surfaces = self._request("/soundsurfaces")
            updated = next((item for item in updated_surfaces if item.get("id") == surface["id"]), None)
            if updated is None:
                raise RuntimeError("Updated SoundMap was not found on the SV600")
            applied = [[item["from"], item["to"]] for item in updated["settings"]["frequencyRanges"]]
            # The SV600 snaps requested frequencies to its measurement bins.
            if len(applied) != len(parsed) or any(
                abs(actual - requested) > 100
                for actual_range, requested_range in zip(applied, parsed)
                for actual, requested in zip(actual_range, requested_range)
            ):
                raise RuntimeError(f"SV600 did not apply the requested frequency range: {applied}")
            self.frequency_ranges = parsed
            self.active_frequency_ranges = applied
            return applied

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
                headers = {"Authorization": "Basic " + base64.b64encode(
                    f"{self.username}:{self.password}".encode()).decode()} if self.username else {}
                request = Request(self.api_url + "/soundsurfaces", headers=headers)
                with urlopen(request, timeout=self.timeout) as response:
                    surfaces = json.load(response)
                surface = next((item for item in surfaces if item.get("settings", {}).get("properties", {}).get("type") == "Data"), None)
                if surface is None:
                    raise RuntimeError("No active SoundMap data surface is available")
                surface_id = surface["id"]
                self.active_frequency_ranges = [
                    [item["from"], item["to"]]
                    for item in surface.get("settings", {}).get("frequencyRanges", [])
                ]
                capture_url = (f"{self.api_url}/data/soundsurface/{surface_id}/capture"
                               "?captureFormat=jpeg&frequencyVisualization=Spectrogram")
                while self._running:
                    start_time = time.monotonic()
                    with urlopen(Request(capture_url, headers=headers), timeout=self.timeout) as response:
                        if not response.headers.get("Content-Type", "").startswith("image/jpeg"):
                            raise ValueError("SoundMap capture did not return a JPEG image")
                        frame = response.read()
                    if not frame.startswith(b"\xff\xd8"):
                        raise ValueError("SoundMap capture returned invalid JPEG data")
                    self.receive_frame(frame, {"format": "jpg", "sound_surface_id": surface_id}, save=self.save_frames)
                    self.is_connected = True
                    remaining = self.capture_interval_sec - (time.monotonic() - start_time)
                    if remaining > 0:
                        time.sleep(remaining)
            except Exception as exc:
                self.is_connected = False
                if self._running:
                    logger.warning(f"[{self.name}] SoundMap capture failed; retrying: {exc}")
                    time.sleep(1)

    def connect(self):
        if not self.enable:
            return False
        self._running = True
        self._thread = threading.Thread(target=self._stream_loop, daemon=True, name=f"{self.name}_stream")
        self._thread.start()
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
                "frequency_ranges": self.active_frequency_ranges or self.frequency_ranges,
                "last_frame_metadata": self.last_frame_metadata}
