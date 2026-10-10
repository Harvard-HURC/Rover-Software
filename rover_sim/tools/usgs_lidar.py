#!/usr/bin/env python3
"""The 0.5 m lidar DEMs rebuilt from USGS 3DEP lidar: fetch_data.py runs the recipe a raster's provenance names
("recipe": "mdrs_area" or "route_area") on the USGS files that provenance lists, which fetch_data downloads and
checks.

mdrs_area makes dem/mdrs_area_lidar2018_0p5m.tif from the 11 tiles of the 2018 UT_Southern_QL1 0.5 m OPR DEM: one
VRT, warped bilinear to EPSG:4326 onto the file's grid (its columns continue the 3DEP lattice at half its pixel,
its rows are square in metres).

route_area makes dem/route_area_lidar_0p5m.tif and its source map dem/route_area_lidar_0p5m_source.tif on the UTM
grid 518000-521000 E x 4250000-4254000 N at 0.5 m: the 2018 OPR DEM (nearest), the class-2 ground returns of the
2020 UT_StatewideSouth point clouds (the mean per cell; the empty cells inside the survey's coverage filled by
normalized Gaussian convolution), the two feathered by distance to each survey's edge, and the cells with neither
filled from dem/route_area_3dep.tif; written as EHdr BIL and warped bilinear (the map: nearest) to EPSG:4326 onto
route_area_3dep.tif's columns at half its pixel.

Both shift the result by the measured (-0.8925, +0.6175) m east, north (the .json's datum_shift_applied): PROJ's
NAD83(2011) -> WGS84 is a null transformation, while the 3DEP and NAIP services apply a real shift. The steps are
the ones that made the files on 2026-10-06, kept as they were (the sigma list, the float64 weights, the EHdr
pixel-centre origin, Python's float strings for -te and -a_ullr, the compound source CRS EPSG:6341+5703 of the DEM
warps: GDAL rounds the 2D EPSG:6341 differently in the last bit); with GDAL 3.13.3, PROJ 9.8.1, OpenCV 4.13.0,
numpy 2.5.3 and laspy 2.7.0 on osx-arm64 they give the files byte for byte (rasters.json's sha256). The warps name
the null coordinate operations PROJ picks offline (-ct) and run with PROJ_NETWORK=OFF, so neither PROJ's network
nor a local grid can add a geoid or a datum shift.
"""
import os
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

NODATA = -999999  # the OPR tiles' no-data value, kept through the warps (R: their GeoTIFF tags)
# [deg] the datum shift (lon, lat): -0.8925 m east, +0.6175 m north (M: phase correlation of hillshades against
# mdrs_area_3dep.tif, 2026-10-06), at 87347.19 and 111004.13 m per degree (38.405 N, GRS80).
SHIFT_DEG = (-0.8925 / 87347.19, 0.6175 / 111004.13)
LON_PIXEL = 5.731922398589246e-06  # [deg] half route_area_3dep.tif's pixel: 0.5007 m east-west at 38.405 N
LAT_PIXEL = 4.5103486609432234e-06  # [deg] the same 0.5007 m north-south
SIGMAS = (0.7, 1.5, 3, 6, 12, 25)  # [cells] the normalized-Gaussian fill's passes, until every cell is filled
BLOCK = 8  # [cells] the 2020 survey's coverage: the 4 m blocks with a ground return, closed by one block
SOURCE_2018, SOURCE_2020, SOURCE_BLEND, SOURCE_3DEP = 1, 2, 3, 4  # the source map's codes (0: none)
ROUTE_FILL = "dem/route_area_3dep.tif"  # the 1 m 3DEP DEM the route grid's cells with no lidar take
# The coordinate operations PROJ 9.8.1 picks offline between NAD83(2011) UTM 12N (EPSG:6341; with NAVD88 heights
# EPSG:6341+5703) and EPSG:4326: null datum transformations (projinfo). The warps name them (gdalwarp -ct), so
# neither PROJ's network (the pixi environment's activation sets PROJ_NETWORK=ON) nor a local grid can add a geoid
# or a datum shift.
UTM_TO_LONLAT = ("+proj=pipeline +step +inv +proj=utm +zone=12 +ellps=GRS80 +step +proj=unitconvert +xy_in=rad "
                 "+xy_out=deg +step +proj=axisswap +order=2,1")
LONLAT_TO_UTM = ("+proj=pipeline +step +proj=axisswap +order=2,1 +step +proj=unitconvert +xy_in=deg +xy_out=rad "
                 "+step +proj=utm +zone=12 +ellps=GRS80")


@dataclass(frozen=True)
class Target:
    """An EPSG:4326 output grid: its edges [deg] and its size [px]."""
    west: float
    south: float
    east: float
    north: float
    cols: int
    rows: int

    def te(self):
        """gdalwarp's -te: the edges moved against the datum shift, so the warp samples the ground that the services
        put at these edges."""
        dlon, dlat = SHIFT_DEG
        return [str(v) for v in (self.west - dlon, self.south - dlat, self.east - dlon, self.north - dlat)]

    def ullr(self):
        """gdal_translate's -a_ullr: the edges themselves."""
        return [repr(v) for v in (self.west, self.north, self.east, self.south)]


@dataclass(frozen=True)
class UtmGrid:
    """A NAD83(2011) UTM 12N grid: the corner (x0, y1) [m] of cell (0, 0) (west, north), the cell size [m], the size
    [cells], and how many rows from the north the 2020 point clouds cover."""
    x0: float
    y1: float
    res: float
    cols: int
    rows: int
    lpc_rows: int

    def te(self):
        return [repr(v) for v in (self.x0, self.y1 - self.rows * self.res, self.x0 + self.cols * self.res, self.y1)]

    def tr(self):
        return [repr(self.res), repr(self.res)]


# The grids of 2026-10-06 (provenance): the bounds as the commands gave them (W, N and the counts; S or E derived).
MDRS_TARGET = Target(-110.8100044091711, 38.38999382716049, -110.76999559082894,
                     38.38999382716049 + 5840 * LAT_PIXEL, 6980, 5840)
ROUTE_TARGET = Target(-110.79, 38.43199735449736 - 6207 * LAT_PIXEL, -110.79 + 4536 * LON_PIXEL,
                      38.43199735449736, 4536, 6207)
ROUTE_GRID = UtmGrid(518000.0, 4254000.0, 0.5, 6000, 8000, 4000)  # the LAZ tiles cover 4252000-4254000 N
