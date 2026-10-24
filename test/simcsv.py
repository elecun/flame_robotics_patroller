#!/usr/bin/env python3
"""
Convert RTK GNSS NMEA CSV log file (e.g., 20261023_135043_rtk.csv)
to MPPI simulation reference path CSV format (x, y, yaw, ref_v) like ovalpath.csv.

Output filename automatically appends '_sim' before the '.csv' extension
(e.g., 20261023_135043_rtk_sim.csv).

By default, the path is aligned to the robot's initial frame at (0, 0, yaw=0.0)
so that the initial travel direction matches +x (yaw = 0 rad), just like ovalpath.csv.
"""

import argparse
import csv
import math
import os
from typing import List, Optional, Tuple

WGS84_EARTH_RADIUS_M = 6378137.0


def parse_nmea_heading_from_raw(raw_nmea: str, sentence_type: str) -> Tuple[Optional[float], Optional[float]]:
    """
    Extract heading (deg, True North = 0, CW positive) and speed (m/s) from raw NMEA string.
    Returns (heading_deg, speed_mps).
    """
    if not raw_nmea:
        return None, None
    body = raw_nmea.strip().split("*")[0]
    parts = body.split(",")
    st = sentence_type.upper()

    heading_deg: Optional[float] = None
    speed_mps: Optional[float] = None

    try:
        if st == "THS" and len(parts) >= 3:
            # $GNTHS,<heading>,<status>*cs (status 'V' means invalid)
            if parts[1] and parts[2] != "V":
                heading_deg = float(parts[1])
        elif st == "HDT" and len(parts) >= 2:
            # $GNHDT,<heading>,T*cs
            if parts[1]:
                heading_deg = float(parts[1])
        elif st == "RMC" and len(parts) >= 9:
            # $GNRMC,<utc>,<status>,<lat>,<N/S>,<lon>,<E/W>,<spd_knots>,<cog_deg>,...
            if len(parts) > 2 and parts[2] == "A":
                if parts[7]:
                    speed_mps = float(parts[7]) * 0.514444
                if parts[8]:
                    heading_deg = float(parts[8])
    except (ValueError, IndexError):
        pass

    return heading_deg, speed_mps


def compass_deg_to_enu_yaw_rad(heading_deg: float) -> float:
    """
    Convert True North clockwise compass heading [0, 360) deg
    to ENU mathematical yaw angle [0, 2*pi) rad (East = 0, North = pi/2, CCW positive).
    """
    enu_deg = (90.0 - heading_deg) % 360.0
    return math.radians(enu_deg)


def latlon_to_enu_meters(
    lat: float, lon: float, lat0: float, lon0: float
) -> Tuple[float, float]:
    """
    Convert WGS84 (latitude, longitude) in degrees to local tangent plane ENU (x=East, y=North) in meters
    relative to origin (lat0, lon0).
    """
    dlat_rad = math.radians(lat - lat0)
    dlon_rad = math.radians(lon - lon0)
    x_east = dlon_rad * WGS84_EARTH_RADIUS_M * math.cos(math.radians(lat0))
    y_north = dlat_rad * WGS84_EARTH_RADIUS_M
    return x_east, y_north


def build_output_path(input_path: str, output_path: Optional[str] = None) -> str:
    """Append '_sim' postfix before file extension if output_path is not explicitly given."""
    if output_path:
        return output_path
    base, ext = os.path.splitext(input_path)
    if not ext:
        ext = ".csv"
    return f"{base}_sim{ext}"


def convert_rtk_csv_to_sim_csv(
    input_csv_path: str,
    output_csv_path: Optional[str] = None,
    ref_v: float = 4.0 / 3.6,
    min_dist: float = 0.05,
    align_initial_heading: bool = True,
) -> str:
    """
    Read RTK log CSV and write simulation reference path CSV (x, y, yaw, ref_v).

    Args:
        input_csv_path: Path to input RTK log CSV (e.g., 20261023_135043_rtk.csv).
        output_csv_path: Optional explicit output path. Defaults to <input>_sim.csv.
        ref_v: Target reference velocity [m/s] (default: 4 km/h = 1.1111 m/s; ovalpath default is 5.0).
        min_dist: Minimum distance [m] between consecutive waypoints to filter stationary GPS jitter.
        align_initial_heading: If True (default), rotate path so initial heading is 0.0 rad (+x direction)
            to match Vehicle.reset() default yaw=0.0 in mppi_pathtracking.ipynb.

    Returns:
        Path to generated output CSV file.
    """
    if not os.path.exists(input_csv_path):
        raise FileNotFoundError(f"Input CSV file not found: {input_csv_path}")

    out_path = build_output_path(input_csv_path, output_csv_path)

    # Raw epoch state
    latest_lat: Optional[float] = None
    latest_lon: Optional[float] = None
    latest_ths_heading_deg: Optional[float] = None
    has_uncommitted_position = False

    raw_samples: List[Tuple[float, float, float]] = []  # (lat, lon, heading_deg)

    with open(input_csv_path, mode="r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            sentence_type = (row.get("sentence_type") or "").strip().upper()
            valid_fix = (row.get("valid_fix") or "").strip().lower() == "true"
            checksum_valid = (row.get("checksum_valid") or "").strip().lower() == "true"
            raw_nmea = row.get("raw_nmea") or ""

            if not checksum_valid:
                continue

            if sentence_type in ("GGA", "RMC") and valid_fix:
                try:
                    lat_val = float(row["latitude"])
                    lon_val = float(row["longitude"])
                    latest_lat = lat_val
                    latest_lon = lon_val
                    has_uncommitted_position = True
                except (ValueError, KeyError, TypeError):
                    pass

            elif sentence_type in ("THS", "HDT"):
                hdg_deg, _ = parse_nmea_heading_from_raw(raw_nmea, sentence_type)
                if hdg_deg is not None:
                    latest_ths_heading_deg = hdg_deg
                    if latest_lat is not None and latest_lon is not None and has_uncommitted_position:
                        raw_samples.append((latest_lat, latest_lon, latest_ths_heading_deg))
                        has_uncommitted_position = False

    # Fallback if log file only had GGA/RMC without THS/HDT sentences
    if not raw_samples:
        with open(input_csv_path, mode="r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                sentence_type = (row.get("sentence_type") or "").strip().upper()
                valid_fix = (row.get("valid_fix") or "").strip().lower() == "true"
                raw_nmea = row.get("raw_nmea") or ""
                if sentence_type == "RMC" and valid_fix:
                    try:
                        lat_val = float(row["latitude"])
                        lon_val = float(row["longitude"])
                        hdg_deg, _ = parse_nmea_heading_from_raw(raw_nmea, "RMC")
                        if hdg_deg is not None:
                            raw_samples.append((lat_val, lon_val, hdg_deg))
                    except (ValueError, KeyError, TypeError):
                        pass

    if not raw_samples:
        raise ValueError(f"No valid position + heading records found in {input_csv_path}")

    lat0, lon0, _ = raw_samples[0]

    # Convert to local ENU meters and filter out stationary duplicate points
    enu_waypoints: List[Tuple[float, float, float, float]] = []
    prev_x: Optional[float] = None
    prev_y: Optional[float] = None

    for lat, lon, hdg_deg in raw_samples:
        x, y = latlon_to_enu_meters(lat, lon, lat0, lon0)
        yaw = compass_deg_to_enu_yaw_rad(hdg_deg)

        if prev_x is None or prev_y is None:
            enu_waypoints.append((x, y, yaw, ref_v))
            prev_x, prev_y = x, y
        else:
            dist = math.hypot(x - prev_x, y - prev_y)
            if dist >= min_dist:
                if len(enu_waypoints) == 1:
                    enu_waypoints[0] = (enu_waypoints[0][0], enu_waypoints[0][1], yaw, ref_v)
                enu_waypoints.append((x, y, yaw, ref_v))
                prev_x, prev_y = x, y

    # Rotate coordinates and yaw by -yaw0 so that initial pose is (0, 0, yaw=0.0 rad)
    # matching ovalpath.csv and Vehicle(init_state=[0, y0, 0, 0])
    waypoints: List[Tuple[float, float, float, float]] = []
    if align_initial_heading and enu_waypoints:
        yaw0 = enu_waypoints[0][2]
        cos_a = math.cos(-yaw0)
        sin_a = math.sin(-yaw0)
        for x, y, yaw, v in enu_waypoints:
            rx = x * cos_a - y * sin_a
            ry = x * sin_a + y * cos_a
            ryaw = (yaw - yaw0) % (2.0 * math.pi)
            waypoints.append((rx, ry, ryaw, v))
    else:
        waypoints = enu_waypoints

    with open(out_path, mode="w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "yaw", "ref_v"])
        for x, y, yaw, v in waypoints:
            writer.writerow([f"{x:.4f}", f"{y:.4f}", f"{yaw:.4f}", f"{v:.4f}"])

    return out_path


def main():
    parser = argparse.ArgumentParser(
        description="Convert RTK NMEA CSV log file to MPPI simulation path CSV (_sim.csv)."
    )
    parser.add_argument(
        "input_csv",
        nargs="?",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "20261023_135043_rtk.csv"),
        help="Input RTK CSV file path (default: ./20261023_135043_rtk.csv)",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Output CSV file path (default: <input_filename>_sim.csv)",
    )
    parser.add_argument(
        "--ref-v",
        type=float,
        default=4.0 / 3.6,
        help="Reference velocity in m/s (default: 1.1111 m/s for 4 km/h)",
    )
    parser.add_argument(
        "--min-dist",
        type=float,
        default=0.05,
        help="Minimum spacing between waypoints in meters to remove stationary jitter (default: 0.05 m)",
    )
    parser.add_argument(
        "--no-align",
        action="store_true",
        help="Keep raw ENU orientation instead of aligning initial heading to 0 rad (+x axis)",
    )
    args = parser.parse_args()

    out_path = convert_rtk_csv_to_sim_csv(
        input_csv_path=args.input_csv,
        output_csv_path=args.output,
        ref_v=args.ref_v,
        min_dist=args.min_dist,
        align_initial_heading=not args.no_align,
    )
    print(f"[INFO] Converted '{args.input_csv}' -> '{out_path}'")


if __name__ == "__main__":
    main()
