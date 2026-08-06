#!/usr/bin/env python3
"""
poi_sampler.py - Sample N uniform POI points from a .route file and generate a .poi file.

Usage:
  python3 poi_sampler.py <route_file> <poi_file> --min 2900 --max 9100 -n 10
  python3 poi_sampler.py -r <route_file> -p <poi_file> --min 2900 --max 9100 -n 10
"""

import sys
import os
import csv
import random
import argparse


def sample_poi_from_route(
    route_path: str,
    poi_path: str,
    min_height: float = 2900.0,
    max_height: float = 9100.0,
    num_samples: int = 10,
) -> None:
    if not os.path.exists(route_path):
        raise FileNotFoundError(f"Route file not found: {route_path}")

    route_data = []

    with open(route_path, "r", encoding="utf-8") as f_in:
        reader = csv.reader(f_in)
        header = next(reader, None)

        lat_idx, lon_idx = 1, 2
        if header:
            for i, col in enumerate(header):
                col_clean = col.strip().lower()
                if col_clean in ["latitude", "lat"]:
                    lat_idx = i
                elif col_clean in ["longitude", "lon", "lng"]:
                    lon_idx = i

        for row in reader:
            if not row or len(row) <= max(lat_idx, lon_idx):
                continue
            try:
                lat = float(row[lat_idx])
                lon = float(row[lon_idx])
                route_data.append((lat, lon))
            except (ValueError, IndexError):
                continue

    if not route_data:
        raise ValueError(f"No valid waypoints found in route file: {route_path}")

    total_pts = len(route_data)
    num_samples = max(1, num_samples)

    if total_pts <= num_samples:
        sampled_data = route_data
    else:
        if num_samples == 1:
            indices = [0]
        else:
            indices = [int(round(i * (total_pts - 1) / (num_samples - 1))) for i in range(num_samples)]
        sampled_data = [route_data[idx] for idx in indices]

    # Ensure parent directory exists
    output_dir = os.path.dirname(os.path.abspath(poi_path))
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    with open(poi_path, "w", encoding="utf-8", newline="") as f_out:
        writer = csv.writer(f_out)
        writer.writerow(["index", "latitude", "longitude", "mast_height", "ptu_pan", "ptu_tilt"])
        for idx, (lat, lon) in enumerate(sampled_data):
            mast_height = round(random.uniform(min_height, max_height), 1)
            writer.writerow([idx, f"{lat:.8f}", f"{lon:.8f}", f"{mast_height:.1f}", 0.0, 0.0])

    print(
        f"Successfully sampled {len(sampled_data)} POI points from '{route_path}' -> '{poi_path}' "
        f"(mast_height range: [{min_height}, {max_height}], ptu_pan=0.0, ptu_tilt=0.0)"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Sample uniform POI points from a .route file and generate a .poi file."
    )
    parser.add_argument("route_pos", nargs="?", help="Input .route file path")
    parser.add_argument("poi_pos", nargs="?", help="Output .poi file path")
    parser.add_argument("-r", "--route", dest="route_opt", help="Input .route file path")
    parser.add_argument("-p", "--poi", dest="poi_opt", help="Output .poi file path")
    parser.add_argument(
        "--min",
        "--min-height",
        dest="min_height",
        type=float,
        default=2900.0,
        help="Minimum mast_height value (default: 2900.0)",
    )
    parser.add_argument(
        "--max",
        "--max-height",
        dest="max_height",
        type=float,
        default=9100.0,
        help="Maximum mast_height value (default: 9100.0)",
    )
    parser.add_argument(
        "-n",
        "--num-samples",
        "--count",
        dest="num_samples",
        type=int,
        default=10,
        help="Number of POI samples to extract (default: 10)",
    )

    args = parser.parse_args()

    route_path = args.route_opt or args.route_pos
    poi_path = args.poi_opt or args.poi_pos

    if not route_path:
        parser.error("Input route file path is required.")
    if not poi_path:
        parser.error("Output poi file path is required.")

    try:
        sample_poi_from_route(
            route_path=route_path,
            poi_path=poi_path,
            min_height=args.min_height,
            max_height=args.max_height,
            num_samples=args.num_samples,
        )
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
