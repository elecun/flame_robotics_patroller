#!/usr/bin/env python3
"""
log2route.py - Convert NMEA GPS log data to APROS .route CSV file format.

Usage:
  python3 log2route.py <input.log> <output.route> [--corridor_boundary 3.0]
  python3 log2route.py -i <input.log> -o <output.route> -c 3.0
"""

import sys
import os
import csv
import argparse


def parse_nmea_lat_lon(lat_raw: str, ns: str, lon_raw: str, ew: str):
    """
    Convert NMEA latitude (DDMM.MMMMMM) and longitude (DDDMM.MMMMMM) to decimal degrees.
    """
    if not lat_raw or not lon_raw:
        raise ValueError("Empty coordinate string")

    # Latitude: DDMM.MMMM...
    lat_deg = float(lat_raw[:2]) + float(lat_raw[2:]) / 60.0
    if ns.upper() == "S":
        lat_deg = -lat_deg

    # Longitude: DDDMM.MMMM...
    lon_deg = float(lon_raw[:3]) + float(lon_raw[3:]) / 60.0
    if ew.upper() == "W":
        lon_deg = -lon_deg

    return lat_deg, lon_deg


def convert_log_to_route(input_path: str, output_path: str, corridor_boundary: float = 3.0) -> None:
    if not os.path.exists(input_path):
        raise FileNotFoundError(f"Input file not found: {input_path}")

    waypoints = []
    seen_timestamps = set()

    with open(input_path, "r", encoding="utf-8", errors="ignore") as f_in:
        for line in f_in:
            line = line.strip()
            if not line:
                continue

            # Remove NMEA checksum if present ($...*CS)
            clean_line = line.split("*")[0]
            parts = clean_line.split(",")
            msg_type = parts[0]

            # Supported NMEA sentences: GNGGA, GPGGA, GNRMC, GPRMC, GNGLL, GPGLL
            if msg_type in ["$GNGGA", "$GPGGA"]:
                # $GNGGA,time,lat,N/S,lon,E/W,fix_quality,...
                if len(parts) >= 6 and parts[2] and parts[4]:
                    time_str = parts[1]
                    if time_str and time_str in seen_timestamps:
                        continue
                    try:
                        lat, lon = parse_nmea_lat_lon(parts[2], parts[3], parts[4], parts[5])
                        waypoints.append((lat, lon))
                        if time_str:
                            seen_timestamps.add(time_str)
                    except ValueError:
                        continue

            elif msg_type in ["$GNRMC", "$GPRMC"]:
                # $GNRMC,time,status,lat,N/S,lon,E/W,...
                if len(parts) >= 7 and parts[2] == "A" and parts[3] and parts[5]:
                    time_str = parts[1]
                    if time_str and time_str in seen_timestamps:
                        continue
                    try:
                        lat, lon = parse_nmea_lat_lon(parts[3], parts[4], parts[5], parts[6])
                        waypoints.append((lat, lon))
                        if time_str:
                            seen_timestamps.add(time_str)
                    except ValueError:
                        continue

            elif msg_type in ["$GNGLL", "$GPGLL"]:
                # $GNGLL,lat,N/S,lon,E/W,time,status,...
                if len(parts) >= 7 and parts[6] == "A" and parts[1] and parts[3]:
                    time_str = parts[5]
                    if time_str and time_str in seen_timestamps:
                        continue
                    try:
                        lat, lon = parse_nmea_lat_lon(parts[1], parts[2], parts[3], parts[4])
                        waypoints.append((lat, lon))
                        if time_str:
                            seen_timestamps.add(time_str)
                    except ValueError:
                        continue

    # Ensure output parent directory exists
    output_dir = os.path.dirname(os.path.abspath(output_path))
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    with open(output_path, "w", encoding="utf-8", newline="") as f_out:
        writer = csv.writer(f_out)
        writer.writerow(["index", "latitude", "longitude", "corridor_boundary"])
        for idx, (lat, lon) in enumerate(waypoints):
            writer.writerow([idx, f"{lat:.8f}", f"{lon:.8f}", corridor_boundary])

    print(
        f"Successfully converted '{input_path}' -> '{output_path}' "
        f"({len(waypoints)} waypoints, corridor_boundary={corridor_boundary})"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Convert NMEA GPS log file to APROS .route CSV file format."
    )
    parser.add_argument("input_pos", nargs="?", help="Input NMEA log file path")
    parser.add_argument("output_pos", nargs="?", help="Output route CSV file path")
    parser.add_argument("-i", "--input", dest="input_opt", help="Input NMEA log file path")
    parser.add_argument("-o", "--output", dest="output_opt", help="Output route CSV file path")
    parser.add_argument(
        "-c",
        "--corridor_boundary",
        "--corridor-boundary",
        type=float,
        default=3.0,
        help="Corridor boundary width in meters (default: 3.0)",
    )

    args = parser.parse_args()

    input_path = args.input_opt or args.input_pos
    output_path = args.output_opt or args.output_pos

    if not input_path:
        parser.error("Input log file path is required.")
    if not output_path:
        parser.error("Output route file path is required.")

    try:
        convert_log_to_route(input_path, output_path, args.corridor_boundary)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
