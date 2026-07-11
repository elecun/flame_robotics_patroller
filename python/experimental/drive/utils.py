import math
from typing import Tuple

# Earth radius in meters
R_EARTH = 6371000.0

def latlon_to_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> Tuple[float, float]:
    """
    Converts two lat/lon coordinates into dx, dy in meters using equirectangular approximation.
    Returns:
        dx, dy: Change in x (Easting) and y (Northing) in meters from point 1 to point 2.
    """
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    
    mean_lat = math.radians((lat1 + lat2) / 2.0)
    
    dy = R_EARTH * dlat
    dx = R_EARTH * dlon * math.cos(mean_lat)
    
    return dx, dy

def meters_to_latlon(lat_ref: float, lon_ref: float, dx: float, dy: float) -> Tuple[float, float]:
    """
    Converts an offset in meters (dx, dy) back to a lat/lon coordinate given a reference point.
    """
    dlat = dy / R_EARTH
    mean_lat = math.radians(lat_ref) + dlat / 2.0
    dlon = dx / (R_EARTH * math.cos(mean_lat))
    
    return lat_ref + math.degrees(dlat), lon_ref + math.degrees(dlon)

def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculates the great-circle distance between two points on the Earth surface.
    """
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    
    a = math.sin(dphi/2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2.0)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R_EARTH * c

def ackermann_steering_angle(dx: float, dy: float, wheelbase: float) -> float:
    """
    Computes a simple pure-pursuit-like steering angle towards a target offset (dx, dy).
    Assumes current heading is aligned with the Y-axis (or X-axis, depending on convention).
    Here we assume the robot's heading is the forward Y axis (dx=lateral, dy=forward).
    """
    if dy == 0 and dx == 0:
        return 0.0
    
    # Distance to target squared
    L_sq = dx**2 + dy**2
    
    # Curvature: kappa = 2 * dx / L^2
    kappa = 2 * dx / L_sq
    
    # Steering angle: delta = atan(kappa * wheelbase)
    delta = math.atan(kappa * wheelbase)
    
    return delta
