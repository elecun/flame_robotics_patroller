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


def convert_rtk_to_route(input_path: str, output_path: str, corridor_boundary: float = 3.0) -> None:
    if not os.path.exists(input_path):
        raise FileNotFoundError(f"Input file not found: {input_path}")

    with open(input_path, "r", encoding="utf-8") as f_in:
        reader = csv.reader(f_in)
        header = next(reader, None)

        # 1st (index 0) & 2nd (index 1) columns: date and time info
        # 3rd (index 2) column: latitude
        # 4th (index 3) column: longitude
        lat_idx, lon_idx = 2, 3

        if header:
            for i, col in enumerate(header):
                col_clean = col.strip().lower()
                if col_clean in ["latitude", "lat"]:
                    lat_idx = i
                elif col_clean in ["longitude", "lon", "lng"]:
                    lon_idx = i

        rows_out = []
        idx = 0

        # If header line itself contained numeric data (no text header), process it first
        if header and len(header) > max(lat_idx, lon_idx):
            try:
                lat = float(header[lat_idx])
                lon = float(header[lon_idx])
                rows_out.append((idx, lat, lon, corridor_boundary))
                idx += 1
            except (ValueError, IndexError):
                pass  # header was indeed text header

        for row in reader:
            if not row or len(row) <= max(lat_idx, lon_idx):
                continue
            try:
                lat = float(row[lat_idx])
                lon = float(row[lon_idx])
                rows_out.append((idx, lat, lon, corridor_boundary))
                idx += 1
            except (ValueError, IndexError):
                continue

    # Ensure output parent directory exists
    output_dir = os.path.dirname(os.path.abspath(output_path))
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    with open(output_path, "w", encoding="utf-8", newline="") as f_out:
        writer = csv.writer(f_out)
        writer.writerow(["index", "latitude", "longitude", "corridor_boundary"])
        for row in rows_out:
            writer.writerow([row[0], f"{row[1]:.8f}", f"{row[2]:.8f}", row[3]])

    print(f"Successfully converted '{input_path}' -> '{output_path}' ({len(rows_out)} waypoints, corridor_boundary={corridor_boundary})")


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
