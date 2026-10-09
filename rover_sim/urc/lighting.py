"""The light of the mission date: the sun's position (NOAA's solar position
algorithm), its colour and intensity, the ambient light, and the clear desert
sky and distance haze that the patched gz-rendering media draw
(sim/tools/gz_media.py writes SKY and the sun into the shaders).

Every URC world stands within 3 km of MDRS and is staged on one day, so all
share MISSION (design spec D13): 2027-05-28 10:30 MDT at 38.418 N, -110.777,
sun elevation 49.8 deg, azimuth 102 deg. Directions are in the world frame
(x east, y north, z up); an SDF directional light's <direction> is the way
the light travels, the opposite of the way to the sun.
"""
import datetime
import math
from dataclasses import dataclass

MDT = datetime.timezone(datetime.timedelta(hours=-6))
MISSION_DATE = datetime.date(2027, 5, 28)  # URC 2027 finals week (design spec D13, A: the day)
MISSION_TIME = datetime.time(10, 30, tzinfo=MDT)  # mid-morning run (A)
MISSION_SITE = (38.418, -110.777)  # [deg] MDRS, the centre of the URC worlds

# The colour is the render prototype's warm high sun (M, sim/data/research/render/manifest.json). The
# intensity makes sunlit flat ground render at its colour map, which is NAIP's apparent colour (design 1:
# rendered map against colour map within CIE76 5): at the prototype's 1.4 the ground came out at 0.77 of
# it, CIE76 8.2; at 1.9 1.006, CIE76 2.1 (M, 2026-10-07, Equipment Servicing's orthophoto, patched media;
# the ambient barely moves it: 0.32 -> 0.50 gave 0.83).
SUN_COLOUR = (1.0, 0.95, 0.87)
SUN_INTENSITY = 1.9
SUN_SPECULAR = (0.3, 0.3, 0.3)  # today's worlds' value, kept (A)
AMBIENT = (0.32, 0.34, 0.40)  # blue sky fill (M, render prototype)
BACKGROUND = (0.62, 0.74, 0.9)  # where nothing is drawn; the sky shader covers it (today's value)


@dataclass(frozen=True)
class Sky:
    """The procedural clear sky and the distance haze of the media patch.
    Colours are sRGB 0-255; the shaders get them in linear light."""
    zenith: tuple = (58, 110, 190)  # deep blue overhead (M: render prototype, tuned by eye on Metal)
    horizon: tuple = (190, 204, 222)  # pale, hazy horizon (M: render prototype)
    horizon_exponent: float = 4.0  # how fast the zenith colour gives way to the horizon's (M)
    ground_tint: tuple = (0.75, 0.72, 0.68)  # below the horizon, x the horizon colour (normally hidden)
    haze_rgb: tuple = None  # haze colour, None: the horizon's
    haze_beta: float = 4.0e-5  # [1/m] extinction: ~98 km visibility (3.9 / beta, M: render prototype)
    haze_max: float = 0.85  # most of a far object the haze replaces

    @property
    def haze(self):
        return self.horizon if self.haze_rgb is None else self.haze_rgb


SKY = Sky()


@dataclass(frozen=True)
class Sun:
    elevation_deg: float
    azimuth_deg: float  # clockwise from north
    colour: tuple = SUN_COLOUR
    intensity: float = SUN_INTENSITY

    @property
    def toward(self):
        """Unit vector from the ground to the sun."""
        el, az = math.radians(self.elevation_deg), math.radians(self.azimuth_deg)
        return (math.sin(az) * math.cos(el), math.cos(az) * math.cos(el), math.sin(el))

    @property
    def direction(self):
        """The way the light travels: an SDF directional light's <direction>."""
        return tuple(-v for v in self.toward)


def sun(date, time, lat, lon):
    """The sun at a date and an aware local time (datetime.time with tzinfo)
    at (lat, lon) [deg], by NOAA's solar position algorithm (the NOAA solar
    calculator spreadsheet; within ~0.01 deg for 1800-2100), with its
    atmospheric refraction. Colour and intensity are the tuned values for a
    high sun (SUN_COLOUR, SUN_INTENSITY): this does not model a low sun."""
    when = datetime.datetime.combine(date, time)
    utc = when.astimezone(datetime.timezone.utc)
    day = utc.toordinal() + 1721424.5  # Julian day at 0 h UTC
    jd = day + (utc.hour + utc.minute / 60 + utc.second / 3600) / 24
    jc = (jd - 2451545.0) / 36525  # Julian centuries since J2000
    l0 = (280.46646 + jc * (36000.76983 + jc * 0.0003032)) % 360  # geometric mean longitude [deg]
    m = 357.52911 + jc * (35999.05029 - 0.0001537 * jc)  # mean anomaly [deg]
    e = 0.016708634 - jc * (0.000042037 + 0.0000001267 * jc)  # orbit eccentricity
    rm = math.radians(m)
    centre = (math.sin(rm) * (1.914602 - jc * (0.004817 + 0.000014 * jc))
              + math.sin(2 * rm) * (0.019993 - 0.000101 * jc) + math.sin(3 * rm) * 0.000289)
    omega = math.radians(125.04 - 1934.136 * jc)
    apparent = math.radians(l0 + centre - 0.00569 - 0.00478 * math.sin(omega))
    obliquity = 23 + (26 + (21.448 - jc * (46.815 + jc * (0.00059 - jc * 0.001813))) / 60) / 60
    obliquity = math.radians(obliquity + 0.00256 * math.cos(omega))
    declination = math.asin(math.sin(obliquity) * math.sin(apparent))
    y = math.tan(obliquity / 2) ** 2
    rl0 = math.radians(l0)
    equation_of_time = 4 * math.degrees(y * math.sin(2 * rl0) - 2 * e * math.sin(rm)
                                        + 4 * e * y * math.sin(rm) * math.cos(2 * rl0)
                                        - 0.5 * y * y * math.sin(4 * rl0) - 1.25 * e * e * math.sin(2 * rm))  # [min]
    minutes = utc.hour * 60 + utc.minute + utc.second / 60
    solar = (minutes + equation_of_time + 4 * lon) % 1440  # true solar time [min]
    hour_angle = math.radians(solar / 4 - 180)
    phi = math.radians(lat)
    cos_zenith = (math.sin(phi) * math.sin(declination)
                  + math.cos(phi) * math.cos(declination) * math.cos(hour_angle))
    zenith = math.acos(max(-1.0, min(1.0, cos_zenith)))
    cos_az = (math.sin(phi) * math.cos(zenith) - math.sin(declination)) / (math.cos(phi) * math.sin(zenith))
    az = math.degrees(math.acos(max(-1.0, min(1.0, cos_az))))
    azimuth = (az + 180) % 360 if hour_angle > 0 else (540 - az) % 360
    elevation = 90 - math.degrees(zenith)
    return Sun(elevation + _refraction(elevation), azimuth)


def _refraction(elevation):
    """Atmospheric refraction [deg] at an apparent-sun elevation (NOAA's approximation)."""
    if elevation > 85:
        return 0.0
    t = math.tan(math.radians(elevation))
    if elevation > 5:
        arcsec = 58.1 / t - 0.07 / t ** 3 + 0.000086 / t ** 5
    elif elevation > -0.575:
        arcsec = 1735 + elevation * (-518.2 + elevation * (103.4 + elevation * (-12.79 + elevation * 0.711)))
    else:
        arcsec = -20.772 / t
    return arcsec / 3600


MISSION = sun(MISSION_DATE, MISSION_TIME, *MISSION_SITE)
