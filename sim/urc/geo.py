"""Local ENU metres <-> WGS84 latitude / longitude / altitude.

Worlds set Gazebo's <spherical_coordinates> to an Origin with
world_frame_orientation ENU and heading 0, so world x is east, y is north and
z is up, and the NavSat sensor reports what enu_to_wgs84 computes. The math is
exact (through ECEF on the WGS84 ellipsoid), like Gazebo's.
"""
import math
from dataclasses import dataclass

A = 6378137.0  # WGS84 semi-major axis [m]
F = 1 / 298.257223563
E2 = F * (2 - F)  # first eccentricity squared


@dataclass(frozen=True)
class Origin:
    lat: float  # [deg]
    lon: float  # [deg]
    alt: float  # [m] above the ellipsoid; Gazebo's <elevation>


def _ecef(lat, lon, alt):
    phi, lam = math.radians(lat), math.radians(lon)
    n = A / math.sqrt(1 - E2 * math.sin(phi) ** 2)
    return ((n + alt) * math.cos(phi) * math.cos(lam),
            (n + alt) * math.cos(phi) * math.sin(lam),
            (n * (1 - E2) + alt) * math.sin(phi))


def _geodetic(x, y, z):
    lam = math.atan2(y, x)
    p = math.hypot(x, y)
    phi = math.atan2(z, p * (1 - E2))
    for _ in range(10):  # converges to well below a millimetre in a few steps
        n = A / math.sqrt(1 - E2 * math.sin(phi) ** 2)
        alt = p / math.cos(phi) - n
        phi = math.atan2(z, p * (1 - E2 * n / (n + alt)))
    n = A / math.sqrt(1 - E2 * math.sin(phi) ** 2)
    return math.degrees(phi), math.degrees(lam), p / math.cos(phi) - n


def _basis(o):
    """East, north and up unit vectors at the origin, in ECEF."""
    phi, lam = math.radians(o.lat), math.radians(o.lon)
    east = (-math.sin(lam), math.cos(lam), 0.0)
    north = (-math.sin(phi) * math.cos(lam), -math.sin(phi) * math.sin(lam), math.cos(phi))
    up = (math.cos(phi) * math.cos(lam), math.cos(phi) * math.sin(lam), math.sin(phi))
    return east, north, up


def enu_to_wgs84(o: Origin, x, y, z=0.0):
    """World position (east, north, up) [m] -> (lat [deg], lon [deg], alt [m])."""
    x0 = _ecef(o.lat, o.lon, o.alt)
    e, n, u = _basis(o)
    p = [x0[i] + x * e[i] + y * n[i] + z * u[i] for i in range(3)]
    return _geodetic(*p)


def wgs84_to_enu(o: Origin, lat, lon, alt=None):
    """(lat, lon[, alt]) -> world position (x, y, z) [m]; alt defaults to the origin's."""
    x0 = _ecef(o.lat, o.lon, o.alt)
    p = _ecef(lat, lon, o.alt if alt is None else alt)
    d = [p[i] - x0[i] for i in range(3)]
    return tuple(sum(d[i] * axis[i] for i in range(3)) for axis in _basis(o))
