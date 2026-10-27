#!/usr/bin/env python3
"""
rtk2route.py - Convert RTK/GPS CSV data to APROS .route CSV file format.

Usage:
  python3 rtk2route.py <input.csv> <output.route> [--corridor_boundary 3.0]
  python3 rtk2route.py -i <input.csv> -o <output.route> -c 3.0
"""

import sys
import os
import csv
import argparse
import itertools


def convert_rtk_to_route(input_path: str, output_path: str, corridor_boundary: float = 3.0) -> int:
    """Convert valid recorded fixes (or legacy latitude/longitude CSV rows)."""
    import math

    with open(input_path, "r", encoding="utf-8-sig", newline="") as source:
        reader = csv.reader(source)
        header = next(reader, None)
        if not header:
            raise ValueError("CSV is empty")
        names = [name.strip().lower() for name in header]
        lat_idx = next((i for i, name in enumerate(names) if name in ("latitude", "lat")), 2)
        lon_idx = next((i for i, name in enumerate(names) if name in ("longitude", "lon", "lng")), 3)
        valid_idx = names.index("valid_fix") if "valid_fix" in names else None
        points = []
        for row in itertools.chain((header,), reader):
            if len(row) <= max(lat_idx, lon_idx):
                continue
            if valid_idx is not None and (len(row) <= valid_idx or row[valid_idx].strip().lower() not in ("true", "1")):
                continue
            try:
                lat, lon = float(row[lat_idx]), float(row[lon_idx])
            except ValueError:
                continue
            if not (math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180):
                continue
            if not points or points[-1] != (lat, lon):
                points.append((lat, lon))
    if len(points) < 2:
        raise ValueError("At least two distinct valid consecutive GNSS fixes are required")
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "x", encoding="utf-8", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(("index", "latitude", "longitude", "corridor_boundary"))
        for index, (lat, lon) in enumerate(points):
            writer.writerow((index, f"{lat:.9f}", f"{lon:.9f}", corridor_boundary))
    print(f"Successfully converted '{input_path}' -> '{output_path}' ({len(points)} waypoints)")
    return len(points)


def main():
    parser = argparse.ArgumentParser(
        description="Convert RTK/GPS CSV log file to APROS .route CSV file format."
    )
    parser.add_argument("input_pos", nargs="?", help="Input CSV file path")
    parser.add_argument("output_pos", nargs="?", help="Output route CSV file path")
    parser.add_argument("-i", "--input", dest="input_opt", help="Input CSV file path")
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
        parser.error("Input CSV file path is required.")
    if not output_path:
        parser.error("Output route file path is required.")

    try:
        convert_rtk_to_route(input_path, output_path, args.corridor_boundary)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
