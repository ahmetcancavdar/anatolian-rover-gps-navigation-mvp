import math
from typing import Tuple

EARTH_RADIUS_M = 6378137.0


def wrap360(angle_deg: float) -> float:
    return angle_deg % 360.0


def wrap180(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def haversine_distance_m(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return EARTH_RADIUS_M * c


def bearing_deg(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    lat1 = math.radians(lat1_deg)
    lat2 = math.radians(lat2_deg)
    dlon = math.radians(lon2_deg - lon1_deg)
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return wrap360(math.degrees(math.atan2(y, x)))


def xy_to_latlon(x_m: float, y_m: float, lat0_deg: float, lon0_deg: float) -> Tuple[float, float]:
    """Approximate local tangent plane conversion for small simulation worlds."""
    lat0 = math.radians(lat0_deg)
    dlat = y_m / EARTH_RADIUS_M
    dlon = x_m / (EARTH_RADIUS_M * math.cos(lat0))
    lat = lat0 + dlat
    lon = math.radians(lon0_deg) + dlon
    return math.degrees(lat), math.degrees(lon)


def quaternion_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    # ROS yaw: CCW from x-axis. For this MVP heading convention, convert in node if needed.
    return math.degrees(yaw)
