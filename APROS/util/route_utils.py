"""
Route Data Utilities (util/route_utils.py)
Provides helper functions for distance-based sampling and processing LiDAR/GNSS route files.
"""

import math
from typing import List, Tuple, Optional


def sample_route_by_distance(
    waypoints: List[Tuple[float, float]],
    corridor_boundaries: Optional[List[float]] = None,
    sample_step_m: float = 1.0
) -> Tuple[List[Tuple[float, float]], List[float]]:
    """
    Sample route waypoints based on cumulative traveled distance (default: 1.0m intervals).
    
    :param waypoints: List of (lat, lon) tuples.
    :param corridor_boundaries: List of corridor boundary width values matching waypoints.
    :param sample_step_m: Target distance step in meters for sampling (default: 1.0m).
    :return: Tuple of (sampled_waypoints, sampled_corridors).
    """
    if not waypoints or len(waypoints) <= 2:
        return waypoints, (corridor_boundaries if corridor_boundaries is not None else [])

    lat0, lon0 = waypoints[0]
    cos_lat0 = math.cos(math.radians(lat0))

    # Compute cumulative distance along the path in meters
    cum_dist = [0.0]
    for i in range(1, len(waypoints)):
        lat_prev, lon_prev = waypoints[i - 1]
        lat_curr, lon_curr = waypoints[i]

        dx = (lat_curr - lat_prev) * 111000.0
        dy = -(lon_curr - lon_prev) * 111000.0 * cos_lat0
        dist = math.hypot(dx, dy)
        cum_dist.append(cum_dist[-1] + dist)

    total_dist = cum_dist[-1]
    if total_dist <= sample_step_m:
        sampled_indices = [0, len(waypoints) - 1]
    else:
        sampled_indices = [0]
        target_d = sample_step_m

        while target_d < total_dist:
            last_idx = sampled_indices[-1]
            if last_idx >= len(waypoints) - 1:
                break

            best_idx = last_idx + 1
            min_err = abs(cum_dist[best_idx] - target_d)

            for idx in range(last_idx + 1, len(waypoints)):
                err = abs(cum_dist[idx] - target_d)
                if err < min_err:
                    min_err = err
                    best_idx = idx
                elif cum_dist[idx] - target_d > min_err + 2.0:
                    break

            if best_idx > last_idx:
                sampled_indices.append(best_idx)

            target_d += sample_step_m

        if sampled_indices[-1] != len(waypoints) - 1:
            sampled_indices.append(len(waypoints) - 1)

    sampled_waypoints = [waypoints[i] for i in sampled_indices]

    if corridor_boundaries is not None and len(corridor_boundaries) == len(waypoints):
        sampled_corridors = [corridor_boundaries[i] for i in sampled_indices]
    else:
        sampled_corridors = []

    return sampled_waypoints, sampled_corridors
