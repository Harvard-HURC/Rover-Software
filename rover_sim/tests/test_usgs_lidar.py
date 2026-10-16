#!/usr/bin/env python3
"""tools/usgs_lidar.py without the network: the grids of 2026-10-06 against rasters.json and the provenance, the
route area's gridding, coverage, fill and feathering on small arrays, the EHdr header, the LAZ reader, and both
recipes end to end on made-up tiles with GDAL's tools, running the GDAL commands of 2026-10-06."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from worldfiles import SIM_DIR

import fetch_data  # noqa: E402  (worldfiles puts tools/ on the path)
import usgs_lidar as L  # noqa: E402
from urc import dem  # noqa: E402

MANIFEST = {e["path"]: e for e in fetch_data.load_manifest()["rasters"]}
DEM_DIR = SIM_DIR / "data" / "dem"
NAN = np.nan
NODATA = "-999999"
# PROJ 9.8.1's null coordinate operations offline (projinfo), which the warps name (gdalwarp -ct).
UTM_TO_LONLAT = ("+proj=pipeline +step +inv +proj=utm +zone=12 +ellps=GRS80 +step +proj=unitconvert +xy_in=rad "
                 "+xy_out=deg +step +proj=axisswap +order=2,1")
LONLAT_TO_UTM = ("+proj=pipeline +step +proj=axisswap +order=2,1 +step +proj=unitconvert +xy_in=deg +xy_out=rad "
                 "+step +proj=utm +zone=12 +ellps=GRS80")


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


def recorded(recipe, *args, **kwargs):
    """(what recipe returns, the GDAL commands it ran), each path in a command as its file name."""
    ran, gdal = [], L.gdal

    def record(*command):
        ran.append([a.name if isinstance(a, Path) else str(a) for a in command])
        gdal(*command)
    with mock.patch.object(L, "gdal", record):
        return recipe(*args, **kwargs), ran


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


class RouteSteps(unittest.TestCase):
    """The route area's gridding, coverage, fill and feathering on small arrays."""

    def test_cell_means_average_and_count_the_points_of_every_tile(self):
        grid = L.UtmGrid(100.0, 200.0, 0.5, 4, 4, 2)
        a = (np.array([100.1, 100.4, 101.2, 99.9, 100.1]), np.array([199.9, 199.6, 199.2, 199.9, 198.9]),
             np.array([10.0, 12.0, 7.0, 50.0, 60.0]))  # the last two lie west of the grid and south of row 1
        b = (np.array([100.2]), np.array([199.8]), np.array([14.0]))
        mean, count = L.cell_means(iter([a, b]), grid)
        want = np.full((2, 4), NAN, np.float32)
        want[0, 0], want[1, 2] = 12.0, 7.0
        np.testing.assert_array_equal(mean, want)
        np.testing.assert_array_equal(count, [[3, 0, 0, 0], [0, 0, 1, 0]])
        self.assertEqual((mean.dtype, count.dtype), (np.float32, np.uint16))

    def test_fill_nan_fills_the_holes_and_keeps_the_rest(self):
        a = np.full((20, 20), 5.0, np.float32)
        a[5, 5] = a[10:14, 10:14] = NAN
        filled = L.fill_nan(a)
        np.testing.assert_allclose(filled, 5.0, atol=1e-5)
        np.testing.assert_array_equal(filled[np.isfinite(a)], a[np.isfinite(a)])
        half = np.full((10, 40), NAN, np.float32)
        half[:, :20] = 3.0  # the far cells need the wide passes
        np.testing.assert_allclose(L.fill_nan(half), 3.0, atol=1e-5)
        full = np.arange(12, dtype=np.float32).reshape(3, 4)
        self.assertIsNot(L.fill_nan(full), full)
        np.testing.assert_array_equal(L.fill_nan(full), full)

    def test_coverage_keeps_the_surveyed_blocks_and_closes_one_block_holes(self):
        count = np.ones((40, 40), np.uint16)
        count[16:24, 16:24] = 0  # the middle block of 5 x 5
        self.assertTrue(L.coverage(count).all())
        count = np.zeros((40, 40), np.uint16)
        count[3, 0:16:8] = 1  # one point in each block of the first two block columns
        want = np.zeros((40, 40), bool)
        want[:8, :16] = True
        np.testing.assert_array_equal(L.coverage(count), want)

    def test_lpc_surface_fills_inside_the_coverage_on_the_north_rows(self):
        grid = L.UtmGrid(0.0, 0.0, 0.5, 24, 24, 16)
        mean = np.full((16, 24), NAN, np.float32)
        mean[:, :8] = 7.0
        mean[3, 3] = NAN
        count = np.zeros((16, 24), np.uint16)
        count[:, :8] = 1
        count[3, 3] = 0
        z20 = L.lpc_surface(mean, count, grid)
        self.assertEqual(z20.shape, (24, 24))
        np.testing.assert_allclose(z20[:16, :8], 7.0, atol=1e-5)  # the empty cell filled
        self.assertTrue(np.isnan(z20[:16, 8:]).all() and np.isnan(z20[16:]).all())

    def test_blend_feathers_where_both_surveys_have_data(self):
        o18 = np.full((3, 12), NAN, np.float32)
        o18[:, :8] = 100.0
        z20 = np.full((3, 12), NAN, np.float32)
        z20[:, 4:] = 101.0
        z, src = L.blend(o18, z20, np.full((3, 12), 50.0, np.float32))
        np.testing.assert_allclose(z[1], [100, 100, 100, 100, 100.2, 100.4, 100.6, 100.8, 101, 101, 101, 101],
                                   atol=1e-5)
        np.testing.assert_array_equal(src[1], [1, 1, 1, 1, 3, 3, 3, 3, 2, 2, 2, 2])
        self.assertEqual(z.dtype, np.float32)

    def test_blend_fills_the_gaps_from_3dep_and_marks_the_rest_no_data(self):
        o18 = np.full((3, 12), NAN, np.float32)
        o18[:, :4] = 100.0
        z20 = np.full((3, 12), NAN, np.float32)
        z20[:, 8:] = 101.0
        dep = np.full((3, 12), NAN, np.float32)
        dep[:, 4:6] = 50.0
        z, src = L.blend(o18, z20, dep)
        np.testing.assert_array_equal(z[1], [100] * 4 + [50] * 2 + [L.NODATA] * 2 + [101] * 4)
        np.testing.assert_array_equal(src[1], [1] * 4 + [4] * 2 + [0] * 2 + [2] * 4)

    def test_write_ehdr_writes_the_headers_of_2026_10_06(self):
        with tempfile.TemporaryDirectory() as d:
            bil = Path(d) / "z.bil"
            L.write_ehdr(bil, np.zeros((2, 3), np.float32), L.ROUTE_GRID)
            self.assertEqual(bil.with_suffix(".hdr").read_text(),
                             "BYTEORDER I\nLAYOUT BIL\nNROWS 8000\nNCOLS 6000\nNBANDS 1\nNBITS 32\nPIXELTYPE FLOAT\n"
                             "ULXMAP 518000.25\nULYMAP 4253999.75\nXDIM 0.5\nYDIM 0.5\nNODATA -999999\n")
            L.write_ehdr(bil, np.array([[1, 2, 3]], np.uint8), L.ROUTE_GRID)
            self.assertEqual(bil.with_suffix(".hdr").read_text(),
                             "BYTEORDER I\nLAYOUT BIL\nNROWS 8000\nNCOLS 6000\nNBANDS 1\nNBITS 8\n"
                             "ULXMAP 518000.25\nULYMAP 4253999.75\nXDIM 0.5\nYDIM 0.5\n")
            self.assertEqual(bil.read_bytes(), b"\x01\x02\x03")

    def test_ground_returns_reads_the_class_2_points_of_a_laz_tile(self):
        with tempfile.TemporaryDirectory() as d:
            grid = L.UtmGrid(518000.0, 4254000.0, 0.5, 2, 2, 1)
            path = laz_tile(Path(d) / "t.laz", grid)
            self.assertEqual(path.read_bytes()[:4], b"LASF")
            x, y, z = L.ground_returns(path)
        self.assertEqual(len(x), 6)  # 3 ground returns in each of 2 cells; the 2 class-1 returns dropped
        np.testing.assert_allclose(sorted(x), [518000.15, 518000.25, 518000.35, 518000.65, 518000.75, 518000.85],
                                   atol=1e-6)
        np.testing.assert_allclose(z, plane(x, y) + 0.01, atol=1e-6)


class Recipes(unittest.TestCase):
    """Both recipes end to end on made-up tiles, with GDAL's tools and laspy. A planar fixture cannot show a change
    in the last bit, so the GDAL commands are pinned too: the DEM warps' compound source CRS EPSG:6341+5703 (M,
    2026-10-10: with the 2D EPSG:6341, 1,879 of OPR tile 12SWH1752's 108,424 interior pixels differ by one float32
    step), the named operations (-ct), the resampling, the no-data value and the creation options give
    rasters.json's files byte for byte."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        (self.dir / "work").mkdir()
        (self.dir / "data" / "dem").mkdir(parents=True)

    def check_gradients(self, z, rows):
        """In `rows` the made-up plane's slopes: 0.04 per m north (rows run south), 0.02 per m east."""
        np.testing.assert_allclose(np.diff(z[rows], axis=0), -0.04 * L.LAT_PIXEL * 111004.13, atol=1e-3)
        np.testing.assert_allclose(np.diff(z[rows], axis=1), 0.02 * L.LON_PIXEL * 87347.19, atol=1e-3)

    def test_route_area_makes_the_dem_and_its_source_map(self):
        grid = L.UtmGrid(518000.0, 4254000.0, 0.5, 48, 96, 48)  # 24 x 48 m; the point cloud covers the north half
        files = {"source_tiles_2018_opr": [opr_tile(self.dir / "opr.tif", 518000.0, 4253988.0, 48, 72)],
                 "source_tiles_2020_lpc": [laz_tile(self.dir / "lpc.laz", grid, holes=[(10, 10)])]}
        fill_raster(self.dir / "data" / L.ROUTE_FILL)
        target = target_at(518002.0, 4253998.0, 36, 80)
        made, ran = recorded(L.route_area, files, self.dir / "data", self.dir / "work", grid=grid, target=target)
        self.assertEqual(sorted(made), ["dem/route_area_lidar_0p5m.tif", "dem/route_area_lidar_0p5m_source.tif"])
        (z,), transform = dem.read_raster(made["dem/route_area_lidar_0p5m.tif"])
        (src,), map_transform = dem.read_raster(made["dem/route_area_lidar_0p5m_source.tif"])
        west, north, east, south = map(float, target.ullr())
        self.assertEqual(transform, (west, (east - west) / 36, 0.0, north, 0.0, (south - north) / 80))
        self.assertEqual(map_transform, transform)
        self.assertEqual((z.shape, src.shape), ((80, 36), (80, 36)))
        # the shift samples 0.62 m south of each pixel: the point cloud alone north of 4253988 N, both surveys
        # down to 4253976 N, the 2018 DEM alone south of it
        self.assertTrue((src[:15] == L.SOURCE_2020).all() and (src[23:38] == L.SOURCE_BLEND).all()
                        and (src[50:] == L.SOURCE_2018).all(), src[:, 0])
        self.assertEqual(set(np.unique(src)), {L.SOURCE_2018, L.SOURCE_2020, L.SOURCE_BLEND})
        self.check_gradients(z, slice(0, 15))  # the filled empty cell among them
        self.check_gradients(z, slice(50, 80))
        self.assertTrue(1400.0 < z.min() and z.max() < 1402.6)  # the class-1 returns, 5 m up, are left out
        utm, size = [*grid.te(), "-tr", "0.5", "0.5"], ["-ts", "36", "80"]
        self.assertEqual(ran, [
            ["gdalbuildvrt", "-q", "-overwrite", "opr2018.vrt", "opr.tif"],
            ["gdalwarp", "-q", "-overwrite", "-te", *utm, "-r", "near", "-dstnodata", NODATA, "opr2018.vrt",
             "opr2018_utm.tif"],
            ["gdalwarp", "-q", "-overwrite", "-s_srs", "EPSG:4326", "-t_srs", "EPSG:6341", "-ct", LONLAT_TO_UTM,
             "-te", *utm, "-r", "bilinear", "route_area_3dep.tif", "fill_utm.tif"],
            ["gdalwarp", "-q", "-overwrite", "-s_srs", "EPSG:6341+5703", "-t_srs", "EPSG:4326", "-ct", UTM_TO_LONLAT,
             "-te", *target.te(), *size, "-r", "bilinear", "-srcnodata", NODATA, "-dstnodata", NODATA, "-ot",
             "Float32", "route_utm.bil", "route_area_lidar_0p5m_warped.tif"],
            ["gdal_translate", "-q", "-a_ullr", *target.ullr(), "-co", "COMPRESS=DEFLATE", "-co", "PREDICTOR=3",
             "-co", "TILED=NO", "route_area_lidar_0p5m_warped.tif", "route_area_lidar_0p5m.tif"],
            ["gdalwarp", "-q", "-overwrite", "-s_srs", "EPSG:6341", "-t_srs", "EPSG:4326", "-ct", UTM_TO_LONLAT,
             "-te", *target.te(), *size, "-r", "near", "route_source_utm.bil",
             "route_area_lidar_0p5m_source_warped.tif"],
            ["gdal_translate", "-q", "-a_ullr", *target.ullr(), "-co", "COMPRESS=DEFLATE",
             "route_area_lidar_0p5m_source_warped.tif", "route_area_lidar_0p5m_source.tif"]])

    def test_route_area_needs_the_3dep_fill(self):
        with self.assertRaisesRegex(FileNotFoundError, "dem/route_area_3dep.tif is not here"):
            L.route_area({}, self.dir / "data", self.dir / "work")

    def test_mdrs_area_warps_the_opr_tiles(self):
        tiles = [opr_tile(self.dir / "a.tif", 518000.0, 4253988.0, 24, 72),
                 opr_tile(self.dir / "b.tif", 518012.0, 4253988.0, 24, 72)]
        target = target_at(518002.0, 4253986.0, 36, 40)
        made, ran = recorded(L.mdrs_area, {"source_tiles": tiles}, self.dir / "data", self.dir / "work", target=target)
        (z,), transform = dem.read_raster(made["dem/mdrs_area_lidar2018_0p5m.tif"])
        west, north, east, south = map(float, target.ullr())
        self.assertEqual(transform, (west, (east - west) / 36, 0.0, north, 0.0, (south - north) / 40))
        self.assertTrue(np.isfinite(z).all() and z.min() > 1400.0)
        self.check_gradients(z, slice(0, 40))
        self.assertEqual(ran, [
            ["gdalbuildvrt", "-q", "-overwrite", "opr2018.vrt", "a.tif", "b.tif"],
            ["gdalwarp", "-q", "-overwrite", "-s_srs", "EPSG:6341+5703", "-t_srs", "EPSG:4326", "-ct", UTM_TO_LONLAT,
             "-te", *target.te(), "-ts", "36", "40", "-r", "bilinear", "-srcnodata", NODATA, "-dstnodata", NODATA,
             "-ot", "Float32", "opr2018.vrt", "mdrs_area_lidar2018_0p5m_warped.tif"],
            ["gdal_translate", "-q", "-a_ullr", *target.ullr(), "-co", "COMPRESS=DEFLATE", "-co", "PREDICTOR=3",
             "-co", "TILED=NO", "mdrs_area_lidar2018_0p5m_warped.tif", "mdrs_area_lidar2018_0p5m.tif"]])


if __name__ == "__main__":
    unittest.main()
