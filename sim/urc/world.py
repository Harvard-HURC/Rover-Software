"""World assembly: one URC world (a mission or a test course) plus its sheet.

Layouts are written in *layout* coordinates: metres east/north/up from the
C2 station (a course's entrance). Gazebo needs the terrain heightmap at the
world origin (the rendered heightmap ignores its model's pose, and DART's
collision ignores the heightmap's <pos>; anything else puts the two apart),
so the world frame is the layout frame shifted to the terrain's centre and
its lowest point. WorldBuilder takes layout coordinates everywhere and writes
world coordinates; the sheet reports world coordinates (what Gazebo's ground
truth and the referee use) and WGS84.

Everything a world is built from is shared: ground types, rock and shrub
colours and the ground textures (terrains.py, urc_media), terrain features
(features.py, terrain.py), and here zones, rocks (scatter, scatter_each,
rock_garden, rock_field), blocks, shrubs, signs and stations. A mission
module only says where and how much. A world's own model holds only what is
unique to it: its heightmap, zone decal meshes and merged rock and shrub
meshes.

Gazebo spends time on every shape every step, touched or not: measured on
the proving ground, 0.56-0.83 us per collision and 0.1-0.18 us per visual
per 1 ms step (400 extra static boxes in the terrain link, out of reach).
Static shapes therefore go in the terrain's own link, merged where they can
be (a body never collides with itself, which keeps them off the broadphase).
"""
import json
import math
import os
import shutil
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import dem, geo, meshes, props, rules, sdf, sheet, terrain, terrains

SYSTEMS = (
    ("gz-sim-physics-system", "gz::sim::systems::Physics", {}),
    ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands", {}),
    ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster", {}),
    ("gz-sim-imu-system", "gz::sim::systems::Imu", {}),
    ("gz-sim-navsat-system", "gz::sim::systems::NavSat", {}),
    ("gz-sim-sensors-system", "gz::sim::systems::Sensors", {"render_engine": "ogre2"}),
)
ROCK_CHUNK = 128.0  # [m] rocks and shrubs are merged into one mesh per square this size
ROCK_BURY = 0.08  # rocks sink this fraction of their size below the ground under their base
SHRUB_SINK = 0.05  # [m] a shrub's origin below the ground
ZONE_OUTLINE_POINTS = 64  # most outline vertices a zone records in the sheet


def garden_spacing(size):
    """Rock garden cell [m] for rocks of `size`: gaps of about a rover width and more."""
    return 0.7 + 3 * size


@dataclass
class Layer:
    """The terrains.TYPES ground `name` as a terrain texture, used from layout
    height z = start upwards and blended in over `fade` metres (the first
    layer covers everything below)."""
    name: str
    start: float = 0.0
    fade: float = 1.0
    tile: float = 8.0  # [m] texture repeat

    @property
    def kind(self):
        return terrains.TYPES[self.name]


class WorldBuilder:
    def __init__(self, key, title, rule, origin: geo.Origin, hf, models_dir, worlds_dir, media, seed, name=None):
        """origin: WGS84 of the layout origin (the C2 station); hf: the terrain
        in layout coordinates; rule: the URC rule the world stages (None for a
        test course); name: the world's name (default urc_<key>)."""
        self.key = key
        self.name = name or f"urc_{key}"
        self.hf = hf
        cx, cy = hf.center
        self.shift = (cx, cy, float(hf.z.min()))
        self.origin = geo.Origin(*geo.enu_to_wgs84(origin, *self.shift))  # the world origin
        self.models_dir = Path(models_dir)
        self.worlds_dir = Path(worlds_dir)
        self.media = media
        self.rng = np.random.default_rng(seed)
        self.root = sdf.model_root("unused")[0]
        self.root.remove(self.root.find("model"))
        self.world = sdf.sub(self.root, "world", name=self.name)
        self.sheet = {"mission": title, **({"rules": f"URC 2027 {rule}"} if rule else {}), "world": self.name,
                      "origin": {"lat": self.origin.lat, "lon": self.origin.lon, "alt": round(self.origin.alt, 3)},
                      "note": "x, y, z: Gazebo world frame [m], x east, y north, z up, the origin above; "
                              "lat/lon WGS84 (what the rover's NavSat reports); alt = origin alt + z.",
                      "points": {}, "objects": {}, "tasks": []}
        self._antenna = None
        self._terrain = None
        self.zones = []
        self._rocks = []  # (world xyz, size, variant, orientation, rgb, collides), merged by write()
        self._shrubs = []  # (world xyz, variant, orientation, rgb), merged by write()
        self._placed = {}  # model name -> layout (x, y): zones may not be declared under them later
        self._setup()

    # --- Coordinates -----------------------------------------------------------------

    def to_world(self, x, y, z):
        cx, cy, cz = self.shift
        return x - cx, y - cy, z - cz

    def to_layout(self, x, y):
        """Layout (x, y) of world (x, y): a sheet entry's."""
        return x + self.shift[0], y + self.shift[1]

    def height(self, x, y):
        """Terrain height at a layout point (layout z): the original surface,
        which the sheet's heightmap and DEM give."""
        return self.hf.height(x, y)

    def ground(self, x, y):
        """What a wheel or an object rests on at layout (x, y): the top of a
        friction zone's tiles, else the terrain."""
        tops = [terrains.top_height(zone.tiles, x, y) for zone in self.zones]
        return max([self.height(x, y)] + [t for t in tops if t is not None])

    def geo(self, x, y, z=None):
        """Sheet entry for a layout point: world x, y, z and WGS84."""
        wx, wy, wz = self.to_world(x, y, self.height(x, y) if z is None else z)
        lat, lon, alt = geo.enu_to_wgs84(self.origin, wx, wy, wz)
        return {"x": round(wx, 3), "y": round(wy, 3), "z": round(wz, 3), "lat": round(lat, 8),
                "lon": round(lon, 8), "alt": round(alt, 2)}

    # --- World basics -------------------------------------------------------------

    def _setup(self):
        w = self.world
        physics = sdf.sub(w, "physics", name="1ms", type="dart")
        sdf.sub(physics, "max_step_size", 0.001)
        sdf.sub(physics, "real_time_factor", 1.0)
        for filename, name, params in SYSTEMS:
            sdf.plugin(w, filename, name, **params)
        scene = sdf.sub(w, "scene")
        sdf.sub(scene, "ambient", (0.5, 0.48, 0.46, 1))
        sdf.sub(scene, "background", (0.62, 0.74, 0.9, 1))
        sdf.sub(scene, "grid", False)
        sdf.sub(scene, "shadows", True)
        sdf.sub(scene, "sky")
        sun = sdf.sub(w, "light", type="directional", name="sun")
        sdf.sub(sun, "cast_shadows", True)
        sdf.pose(sun, (0, 0, 100))
        sdf.sub(sun, "diffuse", (1.0, 0.96, 0.9, 1))
        sdf.sub(sun, "specular", (0.3, 0.3, 0.3, 1))
        sdf.sub(sun, "direction", (-0.45, 0.35, -0.82))
        sph = sdf.sub(w, "spherical_coordinates")
        sdf.sub(sph, "surface_model", "EARTH_WGS84")
        sdf.sub(sph, "world_frame_orientation", "ENU")
        sdf.sub(sph, "latitude_deg", self.origin.lat)
        sdf.sub(sph, "longitude_deg", self.origin.lon)
        sdf.sub(sph, "elevation", self.origin.alt)
        sdf.sub(sph, "heading_deg", 0.0)

    # --- Terrain ---------------------------------------------------------------------

    def terrain(self, layers):
        """The terrain model (heightmap, GeoTIFF DEM, textures) at the world origin.
        At most four layers: Gazebo's ogre2 terrain (Ogre-Next Terra) has four
        detail maps and silently drops the rest. write() writes the heightmap."""
        assert 1 <= len(layers) <= 4, "ogre2 heightmaps blend at most four textures"
        name = f"urc_terrain_{self.key}"
        shutil.rmtree(self.models_dir / name, ignore_errors=True)  # all generated here
        (self.models_dir / name / "meshes").mkdir(parents=True)
        root, model = sdf.model_root(name, static=True)
        link = sdf.link(model, "link")
        col = sdf.sub(link, "collision", name="terrain_collision")
        collision = sdf.sub(sdf.sub(col, "geometry"), "heightmap")
        vis = sdf.sub(link, "visual", name="terrain_visual")
        sdf.sub(vis, "cast_shadows", False)
        visual = sdf.sub(sdf.sub(vis, "geometry"), "heightmap")
        sdf.sub(visual, "use_terrain_paging", False)
        for layer in layers:
            t = sdf.sub(visual, "texture")
            sdf.sub(t, "diffuse", layer.kind.texture(self.media))
            sdf.sub(t, "normal", layer.kind.normal(self.media))
            sdf.sub(t, "size", layer.tile)
        for layer in layers[1:]:
            b = sdf.sub(visual, "blend")
            sdf.sub(b, "min_height", layer.start - self.shift[2])
            sdf.sub(b, "fade_dist", layer.fade)
        # Ground beyond the heightmap: a horizon to look at and a floor that
        # catches anything driven off the edge. The floor is a box: an infinite
        # plane's bounding box overlaps every shape and costs a broadphase pair
        # each step.
        sdf.visual(link, "horizon", sdf.plane((8000, 8000)), (0, 0, -0.3),
                   tuple(0.8 * c / 255 for c in layers[0].kind.rgb), cast_shadows=False)
        sdf.collision(link, "floor", sdf.box((8000, 8000, 1.0)), (0, 0, -2.5))
        # Zones, blocks and decals join this link, rocks and shrubs in write().
        self._terrain = (name, root, link)
        self._heightmaps = (collision, visual)
        self.include(name, "terrain", (0, 0, 0), world=True)

    def _write_terrain(self):
        """heightmap.png (what Gazebo draws and collides with, the sheet's
        terrain) and the same surface as a GeoTIFF DEM. Gazebo scales an
        image heightmap by its own highest pixel (pixel / max_pixel * size_z),
        so the PNG spans 0-65535 from the lowest point (world z = 0) up."""
        name = self._terrain[0]
        directory = self.models_dir / name
        surface = terrain.Heightfield(self.hf.size, self.hf.n, self.hf.z - self.shift[2])
        z_max = float(surface.z.max())
        surface.write_png(directory / "heightmap.png", 0.0, z_max)
        dem.write_geotiff(surface, directory / "dem.tif", self.origin)
        for element in self._heightmaps:
            sdf.sub(element, "uri", sdf.model_uri(name, "heightmap.png"))
            sdf.sub(element, "size", (self.hf.size, self.hf.size, z_max))
        rel = lambda p: os.path.relpath(p, self.worlds_dir)  # noqa: E731
        self.sheet["terrain"] = {"heightmap": rel(directory / "heightmap.png"),
                                 "dem_geotiff": rel(directory / "dem.tif"), "size_m": self.hf.size,
                                 "samples": self.hf.n, "z_max": round(z_max, 4),
                                 "note":"heightmap.png: 16-bit, centred on the world origin, row 0 north, "
                                         "column 0 west; z = pixel / 65535 * z_max"}

    # --- Friction zones ------------------------------------------------------------------

    def zone(self, key, kind, x, y, radius, irregularity=0.3):
        """A zone of terrain type `kind` (terrains.TYPES, whose mu it has): an
        irregular patch up to `radius` around layout (x, y). Below the
        heightmap's mu 1.0 it is a friction zone, its tiles covering the cells
        of a 1.5 m grid whose centres lie inside it; at mu 1.0 only its decal.
        Declare zones before placing anything on them (place() sets objects on
        the tiles, rock_field keeps colliding rocks off them)."""
        seed = int(self.rng.integers(1 << 30))
        return self._add_zone(terrains.blob(key, kind, x, y, radius, seed, irregularity))

    def zone_rect(self, key, kind, x, y, length, width, yaw=0.0, breaks=()):
        """A rectangular zone (test lanes, aprons): `length` along yaw, `width`
        across, centred on layout (x, y); `breaks`: distances from its start
        where the ground has a kink (tile edges go there)."""
        return self._add_zone(terrains.rect(key, kind, x, y, length, width, yaw, breaks))

    def _add_zone(self, zone):
        assert self._terrain is not None, "terrain() first"
        assert zone.key not in {z.key for z in self.zones}, zone.key
        if zone.mu < terrains.HEIGHTMAP_MU:
            terrains.fit_tiles(self.hf, zone)
        for name, (x, y) in self._placed.items():
            if terrains.top_height(zone.tiles, x, y) is not None:
                raise ValueError(f"zone {zone.key} lies under {name}, which was placed before it")
        if zone.tiles:
            tiles = [(zone.tiles, *self._bounds(zone.tiles))]
            for xyz, size, variant, R, _, collides in self._rocks:
                if collides and self._rock_on_tiles(tiles, *self.to_layout(*xyz[:2]), size, variant, R):
                    raise ValueError(f"zone {zone.key} lies under colliding rocks placed before it (rock_field "
                                     "keeps them off zones declared first)")
        model, _, link = self._terrain
        for k, tile in enumerate(zone.tiles):
            sdf.collision(link, f"zone_{zone.key}_{k}", sdf.box(tile.size),
                          (*self.to_world(*tile.center), *sdf.matrix_to_rpy(tile.axes)), mu=zone.mu)
        x0, y0, _ = zone.frame
        d = zone.decal
        if d["shape"] == "blob":
            V, F, UV = meshes.drape(self.hf, x0, y0, d["radius"], d["seed"], offset=terrains.DECAL_OFFSET,
                                    step=d["step"], tile=terrains.DECAL_TILE, irregularity=d["irregularity"],
                                    grow=terrains.DECAL_MARGIN)
        else:
            m = terrains.DECAL_MARGIN
            V, F, UV = meshes.drape_rect(self.hf, x0, y0, d["length"] + 2 * m, d["width"] + 2 * m, d["yaw"],
                                         offset=terrains.DECAL_OFFSET, step=terrains.DECAL_STEP,
                                         tile=terrains.DECAL_TILE, breaks=[b + m for b in d["breaks"]])
        meshes.write_obj(self.models_dir / model / "meshes" / f"zone_{zone.key}.obj", V, F, UV=UV)
        kind = zone.kind
        sdf.visual(link, f"zone_{zone.key}", sdf.mesh(sdf.model_uri(model, "meshes", f"zone_{zone.key}.obj")),
                   self.to_world(x0, y0, 0.0), albedo=kind.texture(self.media), cast_shadows=False)
        self.zones.append(zone)
        stride = max(1, int(math.ceil(len(zone.outline) / ZONE_OUTLINE_POINTS)))
        self.sheet.setdefault("terrain_zones", {})[zone.key] = dict(
            type=kind.key, title=kind.title, mu=kind.mu, center=self.geo(x0, y0), area_m2=round(zone.area, 1),
            tiles=len(zone.tiles), tiled_area_m2=round(zone.tiled_area, 1),
            outline=[[round(v, 2) for v in self.to_world(x, y, 0.0)[:2]] for x, y in zone.outline[::stride]])
        self.sheet.setdefault("terrain_types", {})[kind.key] = dict(
            title=kind.title, mu=kind.mu, max_slope_deg=round(kind.max_slope_deg, 1), notes=kind.notes)
        return zone

    # --- Models -----------------------------------------------------------------------

    def include(self, uri_name, name, xyz, yaw=0.0, static=None, world=False):
        """Include a model at layout position xyz (world position if world=True)."""
        inc = sdf.sub(self.world, "include")
        sdf.sub(inc, "uri", sdf.model_uri(uri_name))
        sdf.sub(inc, "name", name)
        if static is not None:
            sdf.sub(inc, "static", static)
        sdf.pose(inc, (*(xyz if world else self.to_world(*xyz)), 0, 0, yaw))
        if not world:
            self.mark(name, *xyz[:2])
        return inc

    def mark(self, name, x, y):
        """Reserve layout (x, y) as a placed model does: rocks keep clear of it
        (keep_clear) and no zone may be declared over it."""
        self._placed[name] = (x, y)

    def place(self, uri_name, name, x, y, yaw=0.0, dz=0.0, static=None, record=None):
        """Include a model standing on the ground at layout (x, y). Returns layout z."""
        z = self.ground(x, y) + dz
        self.include(uri_name, name, (x, y, z), yaw, static)
        if record is not None:
            self.sheet["objects"][name] = dict(self.geo(x, y, z), model=uri_name, yaw=round(yaw, 4), **record)
        return z

    def ring(self, uri_name, name, center, radius, count):
        """`count` copies of a model (boundary stakes) evenly round a circle,
        each facing the centre, named name_0, name_1, ..."""
        for k in range(count):
            a = 2 * math.pi * k / count
            self.place(uri_name, f"{name}_{k}", center[0] + radius * math.cos(a), center[1] + radius * math.sin(a),
                       a + math.pi)

    def keep_clear(self, paths=(), step=2.0):
        """Layout (x, y) for rocks to keep clear of (their `avoid`): every model
        placed or marked so far, and points every `step` metres along `paths`
        (routes, an approach, a walk)."""
        return list(self._placed.values()) + [tuple(p) for path in paths for p in terrain.resample(path, step)]

    def point(self, key, x, y, z=None, **info):
        self.sheet["points"][key] = dict(self.geo(x, y, z), **info)
        return self.sheet["points"][key]

    def task(self, **fields):
        self.sheet["tasks"].append(fields)

    def station(self, key, x, y, title, description, lines, sign_at, sign_yaw, **info):
        """A test station at layout (x, y): a sheet point with what it tests,
        and a field sign showing `lines` at sign_at (x, y), readable from the
        direction sign_yaw points to."""
        sign = props.field_sign(self.models_dir, self.media, f"{self.key}_{key}", lines)
        self.place(sign, f"sign_{key}", *sign_at, sign_yaw)
        return self.point(key, x, y, title=title, description=description, sign=lines, **info)

    def c2(self, x, y, yaw):
        """C2 station with its antenna mast; records both in the sheet."""
        self.place(props.c2_station(self.models_dir), "c2_station", x, y, yaw)
        ax, ay = props.ANTENNA_OFFSET
        c, s = math.cos(yaw), math.sin(yaw)
        mx, my = x + c * ax - s * ay, y + s * ax + c * ay
        self._antenna = (mx, my, self.height(mx, my) + rules.ANTENNA_MAX_HEIGHT)
        self.sheet["c2"] = dict(self.geo(x, y), yaw=yaw, antenna=self.geo(*self._antenna))

    def rover(self, x, y, yaw):
        """The rover, spawned 2 cm above the highest ground under it."""
        z = max(self.ground(x + dx, y + dy) for dx in (-0.5, 0.5) for dy in (-0.45, 0.45)) + 0.02
        self.include("rover", "rover", (x, y, z), yaw)
        self.sheet["rover_start"] = dict(self.geo(x, y, z), yaw=yaw)

    def radio_los(self, x, y):
        """Line of sight between the C2 antenna and the rover at layout (x, y) (sheet.radio_los)."""
        return sheet.radio_los(self.hf, self._antenna, x, y, self.height(x, y))

    def block(self, name, x, y, yaw, size, top, color):
        """A static box in the terrain's link (features.Step, features.Ledge):
        `size` (along yaw, across, height), its top face at layout height
        `top` above layout (x, y)."""
        sdf.shape(self._terrain[2], name, sdf.box(size), (*self.to_world(x, y, top - size[2] / 2), 0, 0, yaw),
                  color)

    # --- Rocks -----------------------------------------------------------------------------

    def rock_field(self, name, rocks, colliding_size=0.1, palette="desert"):
        """Rocks on the terrain, counted in the sheet as group `name`. rocks:
        [(x, y, size [m], yaw)] in layout coordinates (from scatter or
        scatter_each); size is the long half-axis (about the rock's height);
        rocks smaller than colliding_size are visual only; each takes a colour
        of the terrains.ROCKS palette. A colliding rock that would touch a
        friction zone's tiles is left out: its mesh has mu 1.0 (terrains.py)
        and would give the wheels a foothold there.

        write() merges every group's rocks into meshes in the terrain's own
        link, one per ROCK_CHUNK square (and colour, for the visuals), as
        separate rocks would cost more than the rover (see the module notes).
        Collisions use a coarser mesh of each rock's surface
        (meshes.hull_faces)."""
        colors = terrains.ROCKS[palette]
        zones = [(zone.tiles, *self._bounds(zone.tiles)) for zone in self.zones if zone.tiles]
        kept = []
        for x, y, size, yaw in rocks:
            variant = int(self.rng.integers(meshes.ROCK_VARIANTS))
            color = tuple(colors[int(self.rng.integers(len(colors)))])
            R, z = self._rock_pose(x, y, size, variant, yaw)
            collides = size >= colliding_size
            if collides and zones and self._rock_on_tiles(zones, x, y, size, variant, R):
                continue
            kept.append(collides)
            self._rocks.append((self.to_world(x, y, z), size, variant, R, color, collides))
        group = self.sheet.setdefault("rocks", {}).setdefault(name, {"count": 0, "colliding": 0})
        group["count"] += len(kept)
        group["colliding"] += sum(kept)

    def _rock_pose(self, x, y, size, variant, yaw):
        """Orientation (3x3) and layout z of a rock at layout (x, y): its flat
        base laid on the plane of the ground under it, then sunk until every
        base vertex is ROCK_BURY * size below the ground, so that no edge of
        it floats on a slope."""
        base = meshes.rock_base(variant) * size
        Rz = np.array(sdf.rpy_to_matrix(0.0, 0.0, yaw))
        P = base @ Rz.T
        A = np.column_stack([P[:, :2], np.ones(len(P))])
        (gx, gy, _), *_ = np.linalg.lstsq(A, self.height(x + P[:, 0], y + P[:, 1]), rcond=None)
        R = meshes.tilt(gx, gy) @ Rz
        P = base @ R.T
        return R, float(np.min(self.height(x + P[:, 0], y + P[:, 1]) - P[:, 2])) - ROCK_BURY * size

    @staticmethod
    def _bounds(tiles):
        """Layout bounding box (lo, hi) of tiles, out to their overlap."""
        corners = np.concatenate([t.footprint for t in tiles])
        reach = terrains.TILE_OVERLAP + 1e-6
        return corners.min(axis=0) - reach, corners.max(axis=0) + reach

    @staticmethod
    def _rock_on_tiles(zones, x, y, size, variant, R):
        """True if any vertex of a rock at layout (x, y) (size, variant,
        orientation R) lies over a tile of zones [(tiles, lo, hi)]."""
        V = meshes.rock_variant(variant)[0] * size @ R.T
        x, y = x + V[:, 0], y + V[:, 1]
        for tiles, lo, hi in zones:
            near = (x >= lo[0]) & (x <= hi[0]) & (y >= lo[1]) & (y <= hi[1])
            if near.any() and not np.isnan(terrains.tops(tiles, x[near], y[near])).all():
                return True
        return False

    def _write_clutter(self):
        """Rocks and shrubs, merged into one mesh per ROCK_CHUNK square of the
        world (and colour, for the visuals) in the terrain's link: rock and
        shrub visuals rocks_<i>_<j>_c<k> and shrubs_<i>_<j>_c<k>, rock
        collisions rocks_<i>_<j>."""
        visuals, collisions = defaultdict(list), defaultdict(list)

        def chunk(xyz):
            return math.floor(xyz[0] / ROCK_CHUNK), math.floor(xyz[1] / ROCK_CHUNK)

        def rock(variant, faces, size, R, xyz):
            return (meshes.rock_variant(variant)[0][:faces.max() + 1] * size) @ R.T + xyz, faces

        for xyz, size, variant, R, color, collides in self._rocks:
            # The rock's own 162 vertices for boulders, 42 of them for the rest.
            visuals["rocks", chunk(xyz), color].append(rock(variant, meshes.hull_faces(2 if size >= 0.5 else 1),
                                                            size, R, xyz))
            if collides:
                collisions["rocks", chunk(xyz)].append(rock(variant, meshes.hull_faces(1 if size >= 0.2 else 0),
                                                            size, R, xyz))
        for xyz, variant, R, color in self._shrubs:
            V, F = meshes.shrub(variant)
            visuals["shrubs", chunk(xyz), color].append((V @ R.T + xyz, F))
        palettes = defaultdict(set)
        for group, _, color in visuals:
            palettes[group].add(color)
        for (group, (i, j), color), parts in sorted(visuals.items()):
            k = sorted(palettes[group]).index(color)
            self._write_merged(sdf.visual, f"{group}_{i}_{j}_c{k}", (i, j), parts, color=color)
        for (group, (i, j)), parts in sorted(collisions.items()):
            self._write_merged(sdf.collision, f"{group}_{i}_{j}", (i, j), parts)

    def _write_merged(self, element, name, chunk, parts, **style):
        """Parts [(V, F)] in world coordinates as one mesh, meshes/<name>.obj,
        added to the terrain link as `element` (sdf.visual or sdf.collision)."""
        model, _, link = self._terrain
        V, F = meshes.combine(parts)
        origin = (chunk[0] * ROCK_CHUNK, chunk[1] * ROCK_CHUNK, 0.0)
        meshes.write_obj(self.models_dir / model / "meshes" / f"{name}.obj", V - origin, F)
        element(link, name, sdf.mesh(sdf.model_uri(model, "meshes", f"{name}.obj")), origin, **style)

    def scatter(self, count, center, radius, sizes, avoid=(), clearance=3.0, min_slope=0.0):
        """Random rock placements in a disc, sizes log-uniform in `sizes`,
        keeping `clearance` from the `avoid` points (keep_clear) and, with
        min_slope, only on ground at least that steep [deg] (talus below
        cliffs)."""
        out = []
        tries = 0
        while len(out) < count and tries < count * 50:
            tries += 1
            r = radius * math.sqrt(self.rng.uniform())
            a = self.rng.uniform(0, 2 * math.pi)
            x, y = center[0] + r * math.cos(a), center[1] + r * math.sin(a)
            if any(math.hypot(x - ax, y - ay) < clearance for ax, ay in avoid):
                continue
            if min_slope and self.hf.slope_deg(x, y) < min_slope:
                continue
            size = float(np.exp(self.rng.uniform(math.log(sizes[0]), math.log(sizes[1]))))
            out.append((x, y, size, self.rng.uniform(0, 2 * math.pi)))
        return out

    def scatter_each(self, spots, count, sizes, avoid=(), clearance=3.0, min_slope=0.0):
        """scatter() of `count` rocks in each disc (x, y, radius) of `spots`:
        boulders at several places along a cliff, talus round each hill."""
        return [rock for x, y, radius in spots
                for rock in self.scatter(count, (x, y), radius, sizes, avoid, clearance, min_slope)]

    def scatter_points(self, count, center, radius, avoid=(), clearance=3.0):
        """Random layout (x, y) in a disc (shrubs): scatter()'s placements without the sizes."""
        return [(x, y) for x, y, _, _ in self.scatter(count, center, radius, (1.0, 1.0), avoid, clearance)]

    def points_along(self, path, step, jitter):
        """Layout (x, y) every `step` metres along a polyline, each moved by a
        normal `jitter` [m] (shrubs along a wash)."""
        return [(x + self.rng.normal(0, jitter), y + self.rng.normal(0, jitter))
                for x, y in terrain.resample(path, step)]

    def rock_garden(self, name, x, y, length, width, size, yaw=0.0, spacing=None, avoid=(), clearance=1.0):
        """A dense plot of rocks of about one size, to work the rocker, as rock
        group `name`: one rock per `spacing`-metre cell (default
        garden_spacing) of a length x width rectangle centred on layout (x, y)
        and turned by yaw, jittered so rows do not line up, sizes within
        +-20 % of `size`; every one collides."""
        spacing = garden_spacing(size) if spacing is None else spacing
        nu, nv = max(1, round(length / spacing)), max(1, round(width / spacing))
        c, s = math.cos(yaw), math.sin(yaw)
        out = []
        for i in range(nu):
            for j in range(nv):
                u = -length / 2 + (i + self.rng.uniform(0.2, 0.8)) * length / nu
                v = -width / 2 + (j + self.rng.uniform(0.2, 0.8)) * width / nv
                px, py = x + c * u - s * v, y + s * u + c * v
                rock = (px, py, size * float(np.exp(self.rng.uniform(math.log(0.8), math.log(1.2)))),
                        self.rng.uniform(0, 2 * math.pi))
                if all(math.hypot(px - ax, py - ay) >= clearance for ax, ay in avoid):
                    out.append(rock)
        self.rock_field(name, out, colliding_size=0.0)

    def shrubs(self, points):
        """Desert shrubs at layout points: visual only (the rover drives
        through brush), each a meshes.shrub variant in a terrains.SHRUBS
        colour, merged by write() like the rocks (a model each cost ~3 us per
        step)."""
        for x, y in points:
            variant = int(self.rng.integers(meshes.SHRUB_VARIANTS))
            R = np.array(sdf.rpy_to_matrix(0.0, 0.0, self.rng.uniform(0, 2 * math.pi)))
            color = tuple(terrains.SHRUBS[int(self.rng.integers(len(terrains.SHRUBS)))])
            self._shrubs.append((self.to_world(x, y, self.ground(x, y) - SHRUB_SINK), variant, R, color))
        self.sheet["shrubs"] = len(self._shrubs)

    # --- Output ----------------------------------------------------------------------

    def write(self):
        self._write_terrain()
        self._write_clutter()
        name, root, _ = self._terrain
        sdf.write_model(self.models_dir, name, root, f"Terrain for the URC {self.sheet['mission']} world.")
        self.worlds_dir.mkdir(parents=True, exist_ok=True)
        world_path = self.worlds_dir / f"{self.name}.sdf"
        world_path.write_text(sdf.document(self.root))
        sheet_path = self.worlds_dir / f"{self.name}.json"
        sheet_path.write_text(json.dumps(self.sheet, indent=2) + "\n")
        return world_path, sheet_path
