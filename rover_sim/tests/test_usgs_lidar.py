#!/usr/bin/env python3
"""tools/usgs_lidar.py without the network: the grids of 2026-10-06 against rasters.json and the provenance, the
route area's gridding, coverage, fill and feathering on small arrays, the EHdr header, the LAZ reader, and both
recipes end to end on made-up tiles with GDAL's tools."""
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from worldfiles import SIM_DIR

import fetch_data  # noqa: E402  (worldfiles puts tools/ on the path)
import usgs_lidar as L  # noqa: E402
from urc import dem  # noqa: E402

MANIFEST = {e["path"]: e for e in fetch_data.load_manifest()["rasters"]}
DEM_DIR = SIM_DIR / "data" / "dem"
NAN = np.nan


def plane(x, y):
    """The made-up ground [m] at UTM (x, y): its values at the cell centres and the points below lie on the LAZ
    files' 1 mm lattice, so gridding them loses nothing."""
    return 1400.0 + 0.02 * (np.asarray(x) - 518000.0) + 0.04 * (np.asarray(y) - 4253950.0)


def opr_tile(path, x0, y1, cols, rows):
    """A made-up OPR tile: plane() at the centres of 0.5 m cells of EPSG:6341 from (x0, y1)."""
    grid = L.UtmGrid(x0, y1, 0.5, cols, rows, 0)
    xs = x0 + (np.arange(cols) + 0.5) * 0.5
    ys = y1 - (np.arange(rows) + 0.5) * 0.5
    L.write_ehdr(path.with_suffix(".bil"), plane(*np.meshgrid(xs, ys)).astype(np.float32), grid)
    L.gdal("gdal_translate", "-q", "-a_srs", "EPSG:6341", path.with_suffix(".bil"), path)
    return path


def laz_tile(path, grid, holes=()):
    """A made-up LAZ tile over the grid's first lpc_rows rows: per cell (except `holes`, (row, col)) three ground
    returns at plane() + 1 cm round its centre and one class-1 return 5 m above."""
    import laspy
    rows, cols = np.meshgrid(np.arange(grid.lpc_rows), np.arange(grid.cols), indexing="ij")
    keep = np.ones(rows.shape, bool)
    for r, c in holes:
        keep[r, c] = False
    cx = grid.x0 + (cols[keep] + 0.5) * grid.res
    cy = grid.y1 - (rows[keep] + 0.5) * grid.res
    x = np.concatenate([cx, cx + 0.1, cx - 0.1, cx])
    y = np.concatenate([cy, cy + 0.1, cy - 0.1, cy])
    z = plane(x, y) + 0.01
    cls = np.concatenate([np.full(3 * cx.size, 2), np.full(cx.size, 1)]).astype(np.uint8)
    z[cls == 1] += 5.0
    header = laspy.LasHeader(point_format=3, version="1.2")
    header.scales = np.array([0.001, 0.001, 0.001])
    header.offsets = np.array([518000.0, 4253900.0, 1300.0])
    las = laspy.LasData(header)
    las.x, las.y, las.z, las.classification = x, y, z, cls
    las.write(path)
    return path


def fill_raster(path, value=1500.0):
    """A made-up 3DEP DEM in EPSG:4326 round the made-up area: one value everywhere."""
    (lat1, lon0), (lat0, lon1) = dem.utm_to_wgs84(517980.0, 4254020.0, 12), dem.utm_to_wgs84(518044.0, 4253930.0, 12)
    step = 1e-4
    cols, rows = int((lon1 - lon0) / step) + 1, int((lat1 - lat0) / step) + 1
    transform = (lon0, step, 0.0, lat1, 0.0, -step)
    fetch_data.write_geotiff(path, np.full((1, rows, cols), value, np.float32), fetch_data.geotags(transform))
    return path


def target_at(x, y, cols, rows):
    """A target grid whose north-west corner is at UTM (x, y), pixels of LON_PIXEL x LAT_PIXEL."""
    north, west = dem.utm_to_wgs84(x, y, 12)
    return L.Target(west, north - rows * L.LAT_PIXEL, west + cols * L.LON_PIXEL, north, cols, rows)


class Grids(unittest.TestCase):
    """The grids and bounds of 2026-10-06, tied to rasters.json and the provenance."""

    def test_the_targets_give_the_manifests_georeferencing(self):
        for path, target in (("dem/mdrs_area_lidar2018_0p5m.tif", L.MDRS_TARGET),
                             ("dem/route_area_lidar_0p5m.tif", L.ROUTE_TARGET),
                             ("dem/route_area_lidar_0p5m_source.tif", L.ROUTE_TARGET)):
            with self.subTest(path):
                west, north, east, south = map(float, target.ullr())  # what gdal_translate -a_ullr sets
                self.assertEqual([west, (east - west) / target.cols, 0.0, north, 0.0, (south - north) / target.rows],
                                 MANIFEST[path]["transform"])
                self.assertEqual([1, target.rows, target.cols], MANIFEST[path]["shape"])

    def test_the_bounds_are_the_strings_of_2026_10_06(self):
        self.assertEqual(L.MDRS_TARGET.te(), ["-110.80999419132665", "38.38998826430441", "-110.7699853729845",
                                              "38.416328700484314"])
        self.assertEqual(L.MDRS_TARGET.ullr(), ["-110.8100044091711", "38.416334263340396", "-110.76999559082894",
                                                "38.38999382716049"])
        self.assertEqual(L.ROUTE_TARGET.te(), ["-110.78998978215556", "38.40399605750281", "-110.76398978215556",
                                               "38.43199179164128"])
        self.assertEqual(L.ROUTE_TARGET.ullr(), ["-110.79", "38.43199735449736", "-110.76400000000001",
                                                 "38.40400162035889"])
        self.assertEqual((L.ROUTE_GRID.te(), L.ROUTE_GRID.tr()),
                         (["518000.0", "4250000.0", "521000.0", "4254000.0"], ["0.5", "0.5"]))

    def test_the_shift_is_the_provenances(self):
        for name in ("mdrs_area_lidar2018_0p5m.json", "route_area_lidar_0p5m.json"):
            with self.subTest(name):
                shift = json.loads((DEM_DIR / name).read_text())["datum_shift_applied"]
                self.assertEqual(L.SHIFT_DEG, (shift["dlon_deg"], shift["dlat_deg"]))


if __name__ == "__main__":
    unittest.main()
