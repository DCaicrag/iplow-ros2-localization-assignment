"""Pure mathematical helpers for robot localization.

This module intentionally contains no ROS2 dependencies so that the
coordinate transformations can be unit-tested independently.
"""

import math


WGS84_SEMI_MAJOR_AXIS_M = 6378137.0
WGS84_FLATTENING = 1.0 / 298.257223563
WGS84_ECCENTRICITY_SQUARED = (
    2.0 * WGS84_FLATTENING - WGS84_FLATTENING**2
)


def geodetic_to_enu(
    lat: float,
    lon: float,
    alt: float,
    ref_lat: float,
    ref_lon: float,
    ref_alt: float,
) -> tuple[float, float, float]:
    """Convert WGS84 geodetic coordinates to local ENU coordinates."""
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)

    sin_ref_lat = math.sin(ref_lat_rad)

    denominator = math.sqrt(
        1.0
        - WGS84_ECCENTRICITY_SQUARED
        * sin_ref_lat
        * sin_ref_lat
    )

    prime_vertical_radius = WGS84_SEMI_MAJOR_AXIS_M / denominator

    meridian_radius = (
        prime_vertical_radius
        * (1.0 - WGS84_ECCENTRICITY_SQUARED)
        / (
            1.0
            - WGS84_ECCENTRICITY_SQUARED
            * sin_ref_lat
            * sin_ref_lat
        )
    )

    north = (lat_rad - ref_lat_rad) * meridian_radius

    east = (
        (lon_rad - ref_lon_rad)
        * prime_vertical_radius
        * math.cos(ref_lat_rad)
    )

    up = alt - ref_alt

    return east, north, up


def quaternion_to_yaw(
    x: float,
    y: float,
    z: float,
    w: float,
) -> float:
    """Extract yaw in radians from a ROS quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(siny_cosp, cosy_cosp)