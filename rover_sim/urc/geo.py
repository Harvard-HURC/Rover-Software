"""Local ENU metres <-> WGS84 latitude / longitude / altitude.

Worlds set Gazebo's <spherical_coordinates> to an Origin with
world_frame_orientation ENU and heading 0, so world x is east, y is north and
z is up, and the NavSat sensor reports what enu_to_wgs84 computes. The math is
exact (through ECEF on the WGS84 ellipsoid), like Gazebo's.
"""
import math
from dataclasses import dataclass

import numpy as np

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


def lonlat_fit(o: Origin, size, center=(0.0, 0.0), degree=2, tolerance=1e-9):
    """(x, y) world -> (lat, lon) [deg] over the square of `size` metres
    around `center`, as one vectorised function: a polynomial fit of
    enu_to_wgs84 of total `degree`, fitted on a (2 degree - 1)^2 grid and
    checked between its points (ValueError beyond `tolerance` [deg]). A
    quadratic is within 1e-9 deg over 2 km, where an affine map is 0.1 m off
    at the corners (a degree of longitude shortens northwards); 60 km of far
    field needs a cubic (its quartic terms are ~1 cm at 40 km)."""
    def terms(x, y):
        x, y = (np.asarray(x, float) - center[0]) / size, (np.asarray(y, float) - center[1]) / size
        return np.stack([x ** i * y ** j for i in range(degree + 1) for j in range(degree + 1 - i)], axis=-1)

    def exact(ticks):
        xy = np.array([(center[0] + a * size, center[1] + b * size) for a in ticks for b in ticks])
        return terms(xy[:, 0], xy[:, 1]), np.array([enu_to_wgs84(o, x, y)[:2] for x, y in xy])

    ticks = np.linspace(-0.5, 0.5, 2 * degree - 1)
    coef, *_ = np.linalg.lstsq(*exact(ticks), rcond=None)
    A, check = exact((ticks[1:] + ticks[:-1]) / 2)
    residual = np.abs(A @ coef - check).max()
    if residual > tolerance:
        raise ValueError(f"lat/lon fit is off by {residual:.2e} deg over {size} m")
    return lambda x, y: np.moveaxis(terms(x, y) @ coef, -1, 0)


def wgs84_to_enu(o: Origin, lat, lon, alt=None):
    """(lat, lon[, alt]) -> world position (x, y, z) [m]; alt defaults to the origin's."""
    x0 = _ecef(o.lat, o.lon, o.alt)
    p = _ecef(lat, lon, o.alt if alt is None else alt)
    d = [p[i] - x0[i] for i in range(3)]
    return tuple(sum(d[i] * axis[i] for i in range(3)) for axis in _basis(o))
