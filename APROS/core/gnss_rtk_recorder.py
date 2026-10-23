"""Record received NMEA sentences and export valid fixes as KML and APROS routes."""

import csv
import os
import threading
import xml.etree.ElementTree as ET
from datetime import datetime
from typing import Any, Optional

from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()
CSV_FIELDS = (
    "received_at", "device", "sentence_type", "nmea_utc", "latitude", "longitude",
    "altitude_m", "fix_quality", "satellites", "valid_fix", "checksum_valid", "raw_nmea",
)
KML_NS = "http://www.opengis.net/kml/2.2"


def _coordinate(value: str, hemisphere: str, latitude: bool) -> float:
    if hemisphere not in (("N", "S") if latitude else ("E", "W")):
        raise ValueError("Invalid NMEA hemisphere")
    raw = float(value)
    if raw < 0:
        raise ValueError("Invalid NMEA coordinate")
    degrees = int(raw // 100)
    minutes = raw - degrees * 100
    if minutes >= 60:
        raise ValueError("Invalid NMEA coordinate minutes")
    result = degrees + minutes / 60.0
    if result > (90 if latitude else 180):
        raise ValueError("NMEA coordinate out of range")
    return -result if hemisphere in ("S", "W") else result


def parse_nmea_sentence(line: str) -> dict:
    """Keep every raw sentence, extracting position only from valid GGA/RMC fixes."""
    raw = line.strip()
    if not raw.startswith("$"):
        raise ValueError("Not an NMEA sentence")
    body, marker, checksum = raw[1:].partition("*")
    checksum_valid = False
    if marker:
        try:
            actual = 0
            for character in body:
                actual ^= ord(character)
            checksum_valid = len(checksum) == 2 and actual == int(checksum, 16)
        except ValueError:
            pass
    fields = body.split(",")
    kind = fields[0][-3:] if fields else ""
    row = dict.fromkeys(CSV_FIELDS, "")
    row.update(sentence_type=kind, checksum_valid=checksum_valid, raw_nmea=raw, valid_fix=False)
    if not checksum_valid:
        return row
    try:
        if kind == "GGA" and len(fields) >= 10:
            quality = int(fields[6] or 0)
            row.update(nmea_utc=fields[1], fix_quality=quality, satellites=fields[7], altitude_m=fields[9])
            if quality > 0:
                row.update(latitude=_coordinate(fields[2], fields[3], True),
                           longitude=_coordinate(fields[4], fields[5], False), valid_fix=True)
        elif kind == "RMC" and len(fields) >= 7:
            row["nmea_utc"] = fields[1]
            if fields[2] == "A":
                row.update(latitude=_coordinate(fields[3], fields[4], True),
                           longitude=_coordinate(fields[5], fields[6], False), valid_fix=True)
    except (ValueError, IndexError):
        row.update(latitude="", longitude="", valid_fix=False)
    return row


class GNSSRTKRecorder:
    def __init__(self, robot: Any, output_dir: Optional[str] = None, route_dir: Optional[str] = None):
        self.robot = robot
        apros_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.output_dir = output_dir or os.path.join(apros_root, "datalog")
        self.route_dir = route_dir or os.path.join(apros_root, "route")
        self.is_recording = False
        self.file_path: Optional[str] = None
        self.last_error: Optional[str] = None
        self.last_result: Optional[dict] = None
        self._file = None
        self._writer = None
        self._device = None
        self._lock = threading.Lock()

    def start(self) -> str:
        if self.is_recording:
            return self.file_path
        devices = getattr(self.robot, "devices", {}) or {}
        device = next((devices[name] for name in ("synerex_syn3100", "synerex_syn2000", "synerex_rtk")
                       if name in devices), None)
        if device is None or not hasattr(device, "add_nmea_listener"):
            raise RuntimeError("GNSS/RTK NMEA receiver is unavailable")
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        for index in range(100):
            suffix = f"_{index:02d}" if index else ""
            path = os.path.join(self.output_dir, f"{timestamp}{suffix}_rtk.csv")
            try:
                output = open(path, "x", newline="", encoding="utf-8")
                break
            except FileExistsError:
                continue
        else:
            raise RuntimeError("Could not create a unique GNSS/RTK filename")
        try:
            writer = csv.DictWriter(output, fieldnames=CSV_FIELDS)
            writer.writeheader()
            output.flush()
            with self._lock:
                self._file = output
                self._writer = writer
                self._device = device
                self.file_path = path
                self.last_error = None
                self.last_result = None
                self.is_recording = True
            device.add_nmea_listener(self._on_nmea)
        except Exception:
            with self._lock:
                self.is_recording = False
                self._file = None
                self._writer = None
                self._device = None
            output.close()
            raise
        logger.info(f"[GNSSRTKRecorder] Recording started: {path}")
        return path

    def _on_nmea(self, line: str) -> None:
        try:
            row = parse_nmea_sentence(line)
            row["received_at"] = datetime.now().astimezone().isoformat(timespec="milliseconds")
            row["device"] = getattr(self._device, "name", "")
            with self._lock:
                if self.is_recording and self._writer is not None:
                    self._writer.writerow(row)
                    self._file.flush()
        except Exception as error:
            self.last_error = str(error)
            logger.error(f"[GNSSRTKRecorder] NMEA CSV write failed: {error}")

    def stop(self) -> Optional[dict]:
        with self._lock:
            if not self.is_recording:
                return self.last_result
            self.is_recording = False
            device = self._device
        if device is not None:
            device.remove_nmea_listener(self._on_nmea)
        with self._lock:
            self._file.close()
            self._file = None
            self._writer = None
            self._device = None
            path = self.file_path
        try:
            result = self._export(path)
            self.last_result = result
            logger.info(f"[GNSSRTKRecorder] Recording exported: {result}")
            return result
        except Exception as error:
            self.last_error = str(error)
            logger.error(f"[GNSSRTKRecorder] Export failed: {error}")
            raise

    def _export(self, csv_path: str) -> dict:
        points = []
        raw_count = 0
        with open(csv_path, newline="", encoding="utf-8") as source:
            for row in csv.DictReader(source):
                raw_count += 1
                if row["valid_fix"] == "True":
                    point = (float(row["latitude"]), float(row["longitude"]))
                    if not points or point != points[-1]:
                        points.append(point)
        result = {"csv_path": csv_path, "kml_path": None, "route_path": None,
                  "raw_count": raw_count, "point_count": len(points)}
        if not points:
            return result
        kml_path = os.path.splitext(csv_path)[0] + ".kml"
        root = ET.Element(f"{{{KML_NS}}}kml")
        document = ET.SubElement(root, f"{{{KML_NS}}}Document")
        placemark = ET.SubElement(document, f"{{{KML_NS}}}Placemark")
        ET.SubElement(placemark, f"{{{KML_NS}}}name").text = os.path.basename(csv_path)
        geometry = ET.SubElement(placemark, f"{{{KML_NS}}}{'LineString' if len(points) > 1 else 'Point'}")
        ET.SubElement(geometry, f"{{{KML_NS}}}altitudeMode").text = "clampToGround"
        ET.SubElement(geometry, f"{{{KML_NS}}}coordinates").text = " ".join(
            f"{lon:.9f},{lat:.9f},0" for lat, lon in points)
        temporary = kml_path + ".tmp"
        ET.ElementTree(root).write(temporary, encoding="utf-8", xml_declaration=True)
        os.replace(temporary, kml_path)
        result["kml_path"] = kml_path
        if len(points) < 2:
            return result
        os.makedirs(self.route_dir, exist_ok=True)
        route_path = os.path.join(self.route_dir, os.path.basename(os.path.splitext(csv_path)[0]) + ".route")
        config = getattr(self.robot, "config", None)
        boundary = float(config.get("mobile_drive_s1", "corridor_boundary", fallback=3.0)) if config else 3.0
        temporary = route_path + ".tmp"
        with open(temporary, "w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(("index", "latitude", "longitude", "corridor_boundary"))
            for index, (lat, lon) in enumerate(points):
                writer.writerow((index, f"{lat:.9f}", f"{lon:.9f}", boundary))
        os.replace(temporary, route_path)
        result["route_path"] = route_path
        return result
