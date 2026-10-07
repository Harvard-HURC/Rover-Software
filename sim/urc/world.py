"""World assembly: one URC world (a mission or a test course) plus its sheet.

Layouts are written in *layout* coordinates: metres east/north/up from the
C2 station (a course's entrance). Gazebo needs the terrain heightmap at the
world origin (the rendered heightmap ignores its model's pose, and DART's
collision ignores the heightmap's <pos>; anything else puts the two apart),
so the world frame is the layout frame shifted to the terrain's centre and
its lowest point. WorldBuilder takes layout coordinates everywhere and writes
world coordinates; the sheet reports world coordinates (what Gazebo's ground
truth and the referee use) and WGS84.

Everything a world is built from is shared: ground types and their recipes
(terrains.py: traction, palette, relief, clutter), terrain features
(features.py, terrain.py), paint rules and clutter placement (landscape.py),
colour maps and detail layers (appearance.py), the far field (farfield.py),
the light of the mission date (lighting.py), meshes and textures (meshes.py,
textures.py, urc_media), and here zones, rocks, slabs, risers, shrubs,
pebbles, blocks, signs and stations. A mission module only says where and
how much. A world's own models hold only what is unique to it: its
heightmaps, ground map, colour map, merged clutter meshes and far field.

Gazebo spends time on every shape every step, touched or not: measured on
the proving ground, 0.56-0.83 us per collision and 0.1-0.18 us per visual
per 1 ms step (400 extra static boxes in the terrain link, out of reach).
Static shapes therefore go in the terrain's own link, merged where they can
be (a body never collides with itself, which keeps them off the broadphase):
one mesh per CHUNK square and kind of clutter. Visuals are binary glTF (gate
G3: 0.41x the memory of OBJ), one primitive per colour; collisions are OBJ.

The ground: every world paints a ground raster (landscape.paint: its paint
rules, default DEFAULT_GROUND everywhere, then its zones) and writes it
next to the heightmap as ground.png, with ground.json (legend, traction,
and the collision map naming the ground type of every other shape of the
terrain model), for the drivetrain, the sheet readers and the map: the
rover's drivetrain takes the friction of every wheel contact from them
(design D1). The same raster colours the terrain (appearance.colour_map,
Terra's layer 0, design D11) and says where clutter lies (landscape.place).
The collision heightmap is the visual one carved down by each type's static
sinkage (SINKAGE: the wheels sit 2-3 cm into sand), each PNG normalised to
its own maximum, and objects stand on the carved surface.

Physics: DART at 1 ms steps, its LCP solver chosen per world (design D4;
gates G2 and G5 in sim/data/research): PGS gives each wheel mu times its own
load, which the loaded diagonal of a turn needs, where it runs fast enough;
Dantzig elsewhere, where per-wheel friction is approximate.
"""
import json
import math
import os
import shutil
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from . import appearance, dem, farfield, geo, landscape, lighting, meshes, props, rules, sdf, sheet, terrain, terrains

SYSTEMS = (
    ("gz-sim-physics-system", "gz::sim::systems::Physics", {}),
    ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands", {}),
    ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster", {}),
    ("gz-sim-imu-system", "gz::sim::systems::Imu", {}),
    ("gz-sim-navsat-system", "gz::sim::systems::NavSat", {}),
    ("gz-sim-sensors-system", "gz::sim::systems::Sensors", {"render_engine": "ogre2"}),
    ("gz-sim-particle-emitter-system", "gz::sim::systems::ParticleEmitter", {}),  # the drivetrain's wheel dust
)
CHUNK = 128.0  # [m] clutter is merged into one mesh per square this size and kind
ROCK_BURY = 0.08  # rocks sink this fraction of their size below the ground under their base
SLAB_BURY = 0.15  # slabs sink this fraction of their height (design 5.5)
SLAB_EXPOSED = 0.2  # a tilted slab's top stays this fraction of its thickness above the ground at its buried
# edge (A): tilted further, the buried edge took the top face under the ground (37-40 % of slabs did)
SLAB_COLLIDE = 0.15  # [m] slabs this wide and wider collide (design 5.5)
SHRUB_SINK = 0.05  # [m] a shrub's origin below the ground
PEBBLE_SINK = 0.35  # pebbles sink this fraction of their height (M: render prototype)
IMAGED_CROWN = (0.45, 0.7)  # a shrub's crown / its NAIP dark spot's diameter (M: render prototype, by eye on NAIP)
ZONE_OUTLINE_POINTS = 64  # most outline vertices a zone records in the sheet
SINKAGE = True  # carve the collision heightmap by each ground type's static sinkage (design 5.8)
SINKAGE_EASE = 0.75  # [m] the carve eases in over this inside its type (A: design 0.5-1 m)
DIP = 0.2  # [m] the collision heightmap lies this far under a surface mesh (surface_mesh; A: deeper than the
# 5-15 cm a wheel sinks into a corrugated heightmap, measured)
DIP_EASE = 0.5  # [m] it dips over this inside the mesh's footprint (A)
MAX_SINKAGE = max(t.traction.sinkage_m for t in terrains.TYPES.values())  # [m] world z = 0 lies this far below
# the lowest point when sinkage is on, so that any carve fits above it
COLOUR_TEXELS = 4096  # colour map size (design 5.7: 0.5 m per texel at 2 km, 6.25 cm at 256 m)
CAP_DETAIL = 0.47  # weight slab joints fade in to at the terrain's top, above a world's cap height (M: prototype)
# How each kind of merged clutter looks: roughness (A: matte) and whether it casts shadows (pebbles do not: 20,000
# tiny shadows cost frame time and are not seen, M: render prototype).
STYLE = {"rocks": (0.9, True), "slabs": (0.85, True), "risers": (0.9, True), "shrubs": (0.95, True),
         "pebbles": (0.9, False)}
COLLIDING = ("rocks", "slabs", "risers")  # kinds with collisions (landscape.PREFIXES names their ground)


def site(lat, lon, paths=dem.SITE_DEMS):
    """A world's layout origin at (lat, lon) with the ellipsoidal altitude of
    the ground there (dem.site_altitude: the DEM's NAVD88 elevation made the
    WGS84 height that NavSat and a receiver report), so that every world's
    altitudes are ellipsoidal alike. paths: the DEMs to read."""
    return geo.Origin(lat, lon, dem.site_altitude(lat, lon, paths))


def slab_tilt(tilt, span, height):
    """The tilt [rad] a slab of `span` [m] across its tilt axis and `height`
    keeps: at most `tilt`, and no more than leaves its top SLAB_EXPOSED of
    its height above flat ground at the edge _lay buries (that edge sinks
    span sin t, the slab SLAB_BURY more: span sin t + (SLAB_BURY +
    SLAB_EXPOSED) h <= h cos t)."""
    limit = math.atan2(height, span) - math.asin((SLAB_BURY + SLAB_EXPOSED) * height / math.hypot(height, span))
    return min(tilt, max(limit, 0.0))


def garden_spacing(size):
    """Rock garden cell [m] for rocks of `size`: gaps of about a rover width and more."""
    return 0.7 + 3 * size


def add_relief(hf, rules, features=(), pads=(), paths=(), seed=0, within=None):
    """A synthetic world's micro-relief (design 5.4) added to hf in place:
    the paint rules and the features' footprints say which ground lies where
    (landscape.paint), each type's recipe what relief it carries
    (landscape.relief), kept off engineered ground (landscape.keep_flat:
    features' keep_flat outlines, pads [(x, y, radius)], paths [(polyline,
    half_width)]) and, with `within` (a weight grid in [0, 1]), off
    everything outside it. Returns hf."""
    raster = landscape.paint(hf, rules, features)
    keep = landscape.keep_flat(hf, features, pads, paths)
    if within is not None:
        keep = keep * within
    hf.z += landscape.relief(hf, raster, keep, seed=seed)
    return hf


@dataclass
class Layer:
    """The terrains.TYPES ground `name` as a terrain texture, used from layout
    height z = start upwards and blended in over `fade` metres (the first
    layer covers everything below): the fallback of a terrain without a
    colour map (WorldBuilder.terrain)."""
    name: str
    start: float = 0.0
    fade: float = 1.0
    tile: float = 8.0  # [m] texture repeat

    @property
    def kind(self):
        return terrains.TYPES[self.name]


@dataclass
class Piece:
    """One piece of merged clutter: its vertices in world coordinates, the
    faces of its visual and, if it collides, of its collision mesh (indices
    into the same vertices), and its colour (linear RGB)."""
    V: np.ndarray
    visual: np.ndarray
    collision: np.ndarray
    color: tuple


class WorldBuilder:
    def __init__(self, key, title, rule, origin: geo.Origin, hf, models_dir, worlds_dir, media, seed, name=None,
                 solver=None):
        """origin: WGS84 of the layout origin (the C2 station), its altitude
        ellipsoidal (site()); hf: the terrain in layout coordinates; rule: the
        URC rule the world stages (None for a test course); name: the world's
        name (default urc_<key>); solver: DART's LCP solver (sdf.SOLVERS;
        None: its default, Dantzig)."""
        self.key = key
        self.name = name or f"urc_{key}"
        self.hf = hf
        self.layout_origin = origin  # what landscape.Soils paints the soil map in
        self.sinkage = SINKAGE
        cx, cy = hf.center
        self.shift = (cx, cy, float(hf.z.min()) - (MAX_SINKAGE if self.sinkage else 0.0))
        self.origin = geo.Origin(*geo.enu_to_wgs84(origin, *self.shift))  # the world origin
        self.models_dir = Path(models_dir)
        self.worlds_dir = Path(worlds_dir)
        self.media = media
        self.solver = solver
        self.seed = seed
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
        self._look = None  # how terrain() asked the ground to look (write() makes it)
        self.zones = []
        self._rocks = []  # (world xyz, size, variant, orientation, rgb, collides), merged by write()
        self._pieces = defaultdict(list)  # clutter kind -> [Piece], merged by write()
        self._shrub_discs = []  # layout (x, y, diameter) of the shrubs with meshes
        self._dots = []  # world (x, y, diameter) of shrubs drawn into the colour map, not as meshes
        self._placed = {}  # model name -> layout (x, y): sinking zones may not be declared under them later
        self.legend = landscape.Legend()
        self.paint_rules = [landscape.Base(terrains.DEFAULT_GROUND)]
        self._surfaces = {}  # exact collision name in the terrain link -> ground type key (ground.json)
        self._raster = None  # the ground raster, painted when first needed (ground_map)
        self._carved = None  # the collision surface with sinkage, a layout Heightfield
        self._dips = np.zeros_like(hf.z)  # [m] the collision heightmap under surface meshes (surface_mesh)
        self._margins = []  # (polyline, inner, outer, terrains.Shrubs): where wash-margin shrubs grow
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
        """What a wheel or an object rests on at a layout point (layout z): the
        collision heightmap, the terrain carved by the static sinkage of its
        ground (SINKAGE)."""
        if not self.sinkage:
            return self.height(x, y)
        if self._carved is None:
            self._carved = terrain.Heightfield(self.hf.size, self.hf.n, self.hf.z - self.carve(), self.hf.center)
        return self._carved.height(x, y)

    def geo(self, x, y, z=None):
        """Sheet entry for a layout point: world x, y, z and WGS84."""
        wx, wy, wz = self.to_world(x, y, self.height(x, y) if z is None else z)
        lat, lon, alt = geo.enu_to_wgs84(self.origin, wx, wy, wz)
        return {"x": round(wx, 3), "y": round(wy, 3), "z": round(wz, 3), "lat": round(lat, 8),
                "lon": round(lon, 8), "alt": round(alt, 2)}

    def surface(self):
        """The visual terrain in the world frame, as Gazebo draws it: centred
        on the origin, z from the heightmap's zero."""
        return terrain.Heightfield(self.hf.size, self.hf.n, self.hf.z - self.shift[2])

    # --- World basics -------------------------------------------------------------

    def _setup(self):
        """Physics, systems, the scene and the sun of the mission date
        (lighting.MISSION, design D13), and the world's WGS84 frame."""
        w = self.world
        sdf.physics(w, 0.001, 1.0, self.solver)
        self.sheet["physics"] = {
            "engine": "dart", "step_s": 0.001, "solver": self.solver or "dantzig",
            "note": "pgs gives each wheel mu times its own load; dantzig, DART's default, sizes the friction limits "
                    "from the loads before friction, so per-wheel friction is approximate (realism design D4)"}
        for filename, name, params in SYSTEMS:
            sdf.plugin(w, filename, name, **params)
        scene = sdf.sub(w, "scene")
        sdf.sub(scene, "ambient", (*lighting.AMBIENT, 1))
        sdf.sub(scene, "background", (*lighting.BACKGROUND, 1))
        sdf.sub(scene, "grid", False)
        sdf.sub(scene, "shadows", True)
        sdf.sub(scene, "sky")  # the patched media draw the clear desert sky and haze (lighting.SKY, design D12)
        sun = sdf.sub(w, "light", type="directional", name="sun")
        sdf.sub(sun, "cast_shadows", True)
        sdf.pose(sun, (0, 0, 100))
        sdf.sub(sun, "diffuse", (*lighting.MISSION.colour, 1))
        sdf.sub(sun, "specular", (*lighting.SUN_SPECULAR, 1))
        sdf.sub(sun, "intensity", lighting.MISSION.intensity)
        sdf.sub(sun, "direction", tuple(round(v, 4) for v in lighting.MISSION.direction))
        sph = sdf.sub(w, "spherical_coordinates")
        sdf.sub(sph, "surface_model", "EARTH_WGS84")
        sdf.sub(sph, "world_frame_orientation", "ENU")
        sdf.sub(sph, "latitude_deg", self.origin.lat)
        sdf.sub(sph, "longitude_deg", self.origin.lon)
        sdf.sub(sph, "elevation", self.origin.alt)
        sdf.sub(sph, "heading_deg", 0.0)
        self.sheet["light"] = {"sun_elevation_deg": round(lighting.MISSION.elevation_deg, 2),
                               "sun_azimuth_deg": round(lighting.MISSION.azimuth_deg, 2),
                               "when": f"{lighting.MISSION_DATE} {lighting.MISSION_TIME.strftime('%H:%M')} MDT"}

    # --- Terrain ---------------------------------------------------------------------

    def terrain(self, layers=None, details=None, cap=None, strata=None, orthophoto=None, sources=None,
                texels=COLOUR_TEXELS):
        """The terrain model (heightmap, GeoTIFF DEM, ground map, colour map,
        far field) at the world origin; write() writes its files.

        The ground looks like its ground raster (design D11): a colour map of
        texels x texels, Terra's layer 0, baked from each type's palette
        (appearance.colour_map; strata: {type key: appearance.Strata} for
        the banded types, default badland banding on badland slopes) or, with
        orthophoto (a NAIP GeoTIFF), draped from the imagery, de-shaded
        (appearance.ortho_colour_map). Over it at most three shared detail
        layers: `details` (appearance.DetailLayer), default detail_layers(cap).
        The far field's seam sinks under the terrain's edge (farfield.build).
        sources: provenance for the sheet.

        layers: height-banded textures instead of a colour map (Layer, at most
        four: Ogre-Next Terra has four detail maps and drops the rest), the
        fallback for small test worlds."""
        assert layers is None or 1 <= len(layers) <= 4, "ogre2 heightmaps blend at most four textures"
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
        for layer in layers or ():
            t = sdf.sub(visual, "texture")
            sdf.sub(t, "diffuse", layer.kind.texture(self.media))
            sdf.sub(t, "normal", layer.kind.normal(self.media))
            sdf.sub(t, "size", layer.tile)
        for layer in (layers or ())[1:]:
            b = sdf.sub(visual, "blend")
            sdf.sub(b, "min_height", layer.start - self.shift[2])
            sdf.sub(b, "fade_dist", layer.fade)
        # A floor that catches anything driven off the edge (the far field is
        # what is seen there): a box, as an infinite plane's bounding box
        # overlaps every shape and costs a broadphase pair each step.
        floor = sdf.collision(link, "floor", sdf.box((8000, 8000, 1.0)), (0, 0, -2.5))
        self._surfaces[floor.get("name")] = None  # the world's base ground, known at write()
        # Blocks join this link, clutter in write().
        self._terrain = (name, root, link)
        self._heightmaps = (collision, visual)
        self._look = None if layers else dict(details=details, cap=cap, strata=strata, orthophoto=orthophoto,
                                              texels=texels)
        self._sources = sources
        self.include(name, "terrain", (0, 0, 0), world=True)

    def _write_terrain(self):
        """heightmap.png (what Gazebo draws and, without SINKAGE, collides
        with; the sheet's terrain) and the same surface as a GeoTIFF DEM; with
        SINKAGE the collision surface as heightmap_collision.png. Gazebo
        scales an image heightmap by its own highest pixel (pixel / max_pixel
        * size_z) and does not shift its lowest, so each PNG spans 0-65535
        from world z = 0 (the lowest point, less MAX_SINKAGE with sinkage) to
        its own highest point, which its <size> z names: a carve anywhere,
        the highest point included, stays exact."""
        name = self._terrain[0]
        directory = self.models_dir / name
        surface = self.surface()
        z_max = float(surface.z.max())
        surface.write_png(directory / "heightmap.png", 0.0, z_max)
        dem.write_geotiff(surface, directory / "dem.tif", self.origin)
        rel = lambda p: os.path.relpath(p, self.worlds_dir)  # noqa: E731
        self.sheet["terrain"] = {"heightmap": rel(directory / "heightmap.png"),
                                 "dem_geotiff": rel(directory / "dem.tif"), "size_m": self.hf.size,
                                 "samples": self.hf.n, "z_max": round(z_max, 4),
                                 "note": "heightmap.png: 16-bit, centred on the world origin, row 0 north, "
                                         "column 0 west; z = pixel / 65535 * z_max"}
        sources = self._sources or self._relief_sources()
        if sources:
            self.sheet["terrain"]["sources"] = sources
        collision = ("heightmap.png", z_max)
        if self.sinkage or self._dips.any():
            carved = terrain.Heightfield(self.hf.size, self.hf.n,
                                         surface.z - (self.carve() if self.sinkage else 0.0) - self._dips)
            collision = ("heightmap_collision.png", float(carved.z.max()))
            carved.write_png(directory / collision[0], 0.0, collision[1])
            self.sheet["terrain"].update(collision_heightmap=rel(directory / collision[0]),
                                         z_max_collision=round(collision[1], 4))
        for element, (png, top) in zip(self._heightmaps, (collision, ("heightmap.png", z_max))):
            sdf.sub(element, "uri", sdf.model_uri(name, png))
            sdf.sub(element, "size", (self.hf.size, self.hf.size, top))

    def _relief_sources(self):
        """Where a synthetic world's micro-relief comes from: the lidar
        windows of the relief swatches of the types it paints (design 5.4)."""
        present = np.unique(self.ground_map())
        keys = sorted({self.legend[i].relief.swatch for i in present if self.legend[i].relief.swatch})
        return {"relief": {key: landscape.swatch(key).source for key in keys}} if keys else None

    def detail_layers(self, cap=None):
        """The world's detail layers (appearance.DetailLayer, design 5.7):
        gravel lag and cracked silt at nearly constant weights everywhere
        (appearance.DEFAULT_DETAILS: Terra weights by height only, so a
        detail cannot follow the ground types), then slab joints fading in
        above the layout height `cap` (caprock tops)."""
        out = list(appearance.DEFAULT_DETAILS)
        if cap is not None:
            out.append(appearance.DetailLayer("slab_joints", CAP_DETAIL, above=cap - self.shift[2]))
        return out

    def _write_colour(self):
        """The colour map (Terra's layer 0) and its detail layers on the
        visual heightmap (appearance.terra_layers: the map pre-compensated so
        that the render shows it plus zero-mean detail), and the shrubs that
        have no mesh drawn into it as dots."""
        look = self._look
        name = self._terrain[0]
        directory = self.models_dir / name
        surface = self.surface()
        n = look["texels"]
        if look["orthophoto"]:
            units = self._soil_units(n)
            discs = [(*self.to_world(x, y, 0.0)[:2], d) for x, y, d in self._shrub_discs]
            ortho = appearance.ortho_colour_map(look["orthophoto"], self.origin, self.hf.size, n, dem_hf=surface,
                                                units=units, inpaint_mask=appearance.disc_mask(discs, self.hf.size, n))
            colour = appearance.tint_zones(ortho.rgb, self._zone_texels(n), self.legend.types, self.hf.size)
            self.sheet["terrain"]["orthophoto"] = {
                "source": os.path.relpath(look["orthophoto"], self.worlds_dir),
                "inpainted_share": round(float(ortho.inpainted.mean()), 4),
                "zone_tint": appearance.ZONE_TINT,
                "fitted_sun": {"elevation_deg": round(ortho.sun.elevation_deg, 2),
                               "azimuth_deg": round(ortho.sun.azimuth_deg, 2)}}
        else:
            strata = look["strata"] if look["strata"] is not None else {"badland_slope": appearance.Strata()}
            colour = appearance.colour_map(surface, self.ground_map(), self.legend.types,
                                           np.random.default_rng([self.seed, 1]), n=n, strata=strata,
                                           dots=self._dots)
        details = look["details"] if look["details"] is not None else self.detail_layers(look["cap"])
        layers = appearance.terra_layers(colour, surface, details, self.media)
        Image.fromarray(layers.layer0).save(directory / "colour.png", compress_level=1)
        layers.write(self._heightmaps[1], sdf.model_uri(name, "colour.png"), self.media.flat_normal(),
                     self.hf.size)
        self.sheet["terrain"].update(
            colour_map=os.path.relpath(directory / "colour.png", self.worlds_dir), colour_texels=n,
            details=[dict(key=d.key, weight=d.weight, **({"above_z": round(d.above, 3)} if d.above is not None
                                                         else {})) for d in details],
            colour_clipped=round(layers.clipped, 4))

    def _zone_texels(self, n):
        """The ground type index of every texel of an n x n colour map where a
        zone changed the paint rules' ground (nearest sample), -1 elsewhere."""
        painted = landscape.paint(self.hf, self.paint_rules, (), self.legend)
        index = np.clip(np.floor((np.arange(n) + 0.5) / n * (self.hf.n - 1) + 0.5).astype(int), 0, self.hf.n - 1)
        ground, base = (r[np.ix_(index, index)] for r in (self.ground_map(), painted))
        return np.where(ground != base, ground.astype(np.int32), -1)

    def _soil_units(self, n):
        """SSURGO map unit of every texel of an n x n colour map (deshade
        fits NAIP's shading per unit), from the soil map the paint rules
        name, else one unit."""
        soils = next((rule for rule in self.paint_rules if isinstance(rule, landscape.Soils)), None)
        if soils is None:
            return None
        found, _ = soils.unit_map(landscape.Canvas(self.hf, self.legend))
        index = np.clip(np.floor((np.arange(n) + 0.5) / n * (self.hf.n - 1) + 0.5).astype(int), 0, self.hf.n - 1)
        return found[np.ix_(index, index)].astype(np.int32)

    def _write_farfield(self):
        """The far field (farfield.build, design D14): the real landscape
        out to 40 km, its seam sunk under the terrain's edge."""
        name = farfield.build(self.models_dir, self.media, f"urc_farfield_{self.key}", self.origin, self.surface())
        self.include(name, "farfield", (0, 0, 0), world=True)

    # --- Ground ------------------------------------------------------------------------

    def paint(self, rules):
        """The world's paint rules (landscape.paint), instead of
        DEFAULT_GROUND everywhere; before anything needs the ground raster."""
        assert self._raster is None, "paint() before the ground raster is used"
        self.paint_rules = list(rules)

    def ground_map(self):
        """The ground raster on the heightmap's grid (uint8 indices into
        self.legend): the paint rules, then every zone declared so far."""
        if self._raster is None:
            self._raster = landscape.paint(self.hf, self.paint_rules, self.zones, self.legend)
        return self._raster

    def carve(self):
        """How far [m] the collision surface lies below the terrain at each
        sample: the static sinkage of its ground type (design 5.8), eased in
        over SINKAGE_EASE inside the type (a minimum filter, then a box blur
        of the same size: never deeper than the type's own sinkage, nothing
        outside it)."""
        raster = self.ground_map()  # first: painting adds the types the catalogue lacks to the legend
        cut = np.array([t.traction.sinkage_m for t in self.legend.types], np.float32)[raster]
        k = 2 * int(round(SINKAGE_EASE / 2 / self.hf.res)) + 1
        if k > 1:
            cut = cv2.blur(cv2.erode(cut, np.ones((k, k), np.uint8)), (k, k))
        return cut.astype(float)

    def _write_ground(self):
        """ground.png (the ground raster, 8-bit) and ground.json (legend,
        traction, collision map: design 9.1) next to the heightmap."""
        name, _, link = self._terrain
        directory = self.models_dir / name
        Image.fromarray(self.ground_map()).save(directory / "ground.png")  # uint8: 8-bit grey
        base = next((rule.key for rule in reversed(self.paint_rules) if isinstance(rule, landscape.Base)),
                    terrains.DEFAULT_GROUND)
        collisions = {}
        for c in link.findall("collision"):
            key = c.get("name")
            if key == "terrain_collision" or any(key.startswith(prefix) for prefix in landscape.PREFIXES):
                continue
            if key not in self._surfaces:
                raise ValueError(f"the terrain's collision {key} has no ground type for ground.json")
            collisions[key] = self._surfaces[key] or base
        info = landscape.ground_json(self.legend, self.hf.size, self.hf.n, base, collisions)
        (directory / "ground.json").write_text(json.dumps(info, indent=1) + "\n")
        raster = self.ground_map()
        share = np.bincount(raster.ravel(), minlength=len(self.legend)) / raster.size
        self.sheet["terrain"].update(ground_map=os.path.relpath(directory / "ground.png", self.worlds_dir),
                                     ground_legend=os.path.relpath(directory / "ground.json", self.worlds_dir),
                                     ground_share={self.legend[i].key: round(float(s), 4)
                                                   for i, s in enumerate(share) if s > 0})
        for i in np.flatnonzero(share):  # the painted types too, not only the zones'
            self._type_entry(self.legend[int(i)])

    # --- Zones -------------------------------------------------------------------------

    def zone(self, key, kind, x, y, radius, irregularity=0.3):
        """A zone of terrain type `kind` (terrains.TYPES): an irregular patch
        up to `radius` around layout (x, y). It paints the ground raster,
        which tells the drivetrain how the ground grips and the colour map
        how it looks. Declare zones before placing anything on them: a zone
        whose ground sinks moves the ground under what stands there (place()
        sets objects on the carved ground)."""
        seed = int(self.rng.integers(1 << 30))
        return self._add_zone(terrains.blob(key, kind, x, y, radius, seed, irregularity))

    def zone_rect(self, key, kind, x, y, length, width, yaw=0.0):
        """A rectangular zone (test lanes, aprons): `length` along yaw, `width`
        across, centred on layout (x, y)."""
        return self._add_zone(terrains.rect(key, kind, x, y, length, width, yaw))

    def _add_zone(self, zone):
        assert self._terrain is not None, "terrain() first"
        assert zone.key not in {z.key for z in self.zones}, zone.key
        self._raster = self._carved = None  # the ground changes
        if self.sinkage and zone.kind.traction.sinkage_m:
            for name, (x, y) in self._placed.items():
                if terrains.inside(zone.outline, x, y):
                    raise ValueError(f"zone {zone.key} would sink the ground under {name}, which was placed "
                                     "before it")
        self.zones.append(zone)
        kind = zone.kind
        x0, y0, _ = zone.frame
        stride = max(1, int(math.ceil(len(zone.outline) / ZONE_OUTLINE_POINTS)))
        self.sheet.setdefault("terrain_zones", {})[zone.key] = dict(
            type=kind.key, title=kind.title, center=self.geo(x0, y0), area_m2=round(zone.area, 1),
            outline=[[round(v, 2) for v in self.to_world(x, y, 0.0)[:2]] for x, y in zone.outline[::stride]])
        self._type_entry(kind)
        return zone

    def _type_entry(self, kind):
        """The sheet's terrain_types entry of a ground type: its traction (under terrains.DIG) and notes. Every
        type a zone declares or the ground map holds has one."""
        traction = terrains.traction(kind)
        self.sheet.setdefault("terrain_types", {})[kind.key] = dict(
            title=kind.title, mu_s=traction.mu_s, mu_k=traction.mu_k, climb_deg=round(traction.climb_deg, 1),
            hold_deg=round(traction.hold_deg, 1), sinkage_m=traction.sinkage_m, notes=kind.notes)

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
        (keep_clear) and no sinking zone may be declared over it."""
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

    def block(self, name, x, y, yaw, size, top, color, surface=terrains.TERRAIN_SURFACE):
        """A static box in the terrain's link (features.Step, features.Ledge):
        `size` (along yaw, across, height), its top face at layout height
        `top` above layout (x, y); its ground type in ground.json is
        `surface`."""
        sdf.shape(self._terrain[2], name, sdf.box(size), (*self.to_world(x, y, top - size[2] / 2), 0, 0, yaw),
                  color)
        self._surfaces[f"{name}_collision"] = surface

    def surface_mesh(self, name, vertices, faces, footprint, surface=terrains.DEFAULT_GROUND):
        """A collision mesh in the terrain's link for ground DART's heightmap
        collision cannot carry (features.Washboard: on corrugations its
        cylinder wheels sink 5-15 cm into the heightmap and stall, measured;
        on a mesh they ride the crests). vertices: layout (x, y, z) on the
        ground (ground()); faces: triangles. Within the layout polygon
        `footprint` the collision heightmap dips DIP below the ground, eased
        in over DIP_EASE, so the wheels touch only the mesh; the heightmap
        still draws the ground. ground.json names its ground `surface`."""
        model, _, link = self._terrain
        V = np.array([self.to_world(x, y, z) for x, y, z in vertices])
        meshes.write_obj(self.models_dir / model / "meshes" / f"{name}.obj", V, np.asarray(faces))
        sdf.collision(link, name, sdf.mesh(sdf.model_uri(model, "meshes", f"{name}.obj")))
        self._surfaces[f"{name}_collision"] = surface
        inside = landscape.Canvas(self.hf, self.legend).polygon(footprint).astype(np.float32)
        k = 2 * int(round(DIP_EASE / 2 / self.hf.res)) + 1
        ease = cv2.blur(cv2.erode(inside, np.ones((k, k), np.uint8)), (k, k))
        self._dips = np.maximum(self._dips, DIP * ease)

    # --- Clutter: rocks, slabs, risers, shrubs, pebbles --------------------------------------

    def wash_margin(self, path, inner, outer, recipe):
        """The margins of a wash (features.Wash): the bands from `inner` to
        `outer` metres either side of the layout polyline `path`, where the
        Shrubs `recipe` grows (design 5.5, D10: 0.5-2.5 m tall on the
        lidar's cutbanks), not a ground type of their own: clutter() places
        them, imaged_shrubs() sizes the shrubs found there by it."""
        self._margins.append((tuple(map(tuple, path)), inner, outer, recipe))

    def _in_margins(self):
        """[(boolean grid of a wash margin's band, its Shrubs recipe)] on the heightmap's samples."""
        canvas = landscape.Canvas(self.hf, self.legend)
        return [(canvas.stroke(path, outer) & ~canvas.stroke(path, inner), recipe)
                for path, inner, outer, recipe in self._margins]

    def near(self, spots=(), paths=()):
        """A boolean grid on the heightmap's samples: within radius of the
        layout points spots [(x, y, radius)] or half_width of the polylines
        paths [(polyline, half_width)] (where clutter goes: design D10, a
        mission's radius)."""
        canvas = landscape.Canvas(self.hf, self.legend)
        out = np.zeros((self.hf.n, self.hf.n), np.uint8)
        for x, y, radius in spots:
            (c, r), = canvas.pixels([(x, y)])
            cv2.circle(out, (int(round(c * 16)), int(round(r * 16))), int(round(radius / self.hf.res * 16)), 1, -1,
                       shift=4)
        mask = out.astype(bool)
        for path, half_width in paths:
            mask |= canvas.stroke(path, half_width)
        return mask

    def clutter(self, within=None, avoid=(), clearance=3.0, rock_sizes=None, slab_sizes=None, rocks_within=None,
                shrubs=True, shrubs_3d=None):
        """The ground's own clutter by the catalogue's recipes (design 5.5,
        landscape.place on the ground raster with its zones): slabs and
        risers within `within` (a boolean grid on the heightmap's samples,
        default everywhere), rocks within rocks_within (default `within`),
        all `clearance` metres clear of the layout points `avoid`
        (keep_clear); with `shrubs`, shrubs everywhere, as meshes within
        shrubs_3d (default `within`) and as dots in the colour map elsewhere
        (design D10; a world on real ground has its own: imaged_shrubs).
        rock_sizes, slab_sizes: (low, high) [m] instead of the recipes'
        ranges (rocks: the clutter budget; slabs: real-DEM worlds take
        0.15-1 m, design D9). Groups "slabs", "risers" and "gravel" in the
        sheet, with the slabs' size-frequency per ground type (clutter_report)
        and the shrubs' density, with the wash margins' (wash_margin) as
        "wash_margin"."""
        raster = self.ground_map()
        rng = np.random.default_rng([self.seed, 2])  # its own stream: the world's other placements do not move it

        def place(kind, **extra):
            return landscape.place(self.hf, raster, kind, rng, avoid, clearance, self.legend, **extra)

        slabs = place("slabs", within=within, sizes=slab_sizes)
        self.slabs("slabs", slabs)
        allowed = ~landscape.avoid_mask(self.hf, avoid, clearance)
        self.sheet["slabs"]["slabs"]["by_type"] = self.clutter_report(
            slabs, allowed if within is None else allowed & np.asarray(within, bool))
        self.risers("risers", place("risers", within=within))
        rocks = place("rocks", within=within if rocks_within is None else rocks_within, sizes=rock_sizes)
        self.rock_field("gravel", [(p.x, p.y, p.size, p.yaw) for p in rocks])
        if not shrubs:
            return
        placed = place("shrubs")
        density = {key: dict(entry, recipe_per_ha=self.legend[self.legend.index(key)].clutter.shrubs.per_ha)
                   for key, entry in self.clutter_report(placed, np.ones(raster.shape, bool), sizes=False).items()}
        for band, recipe in self._in_margins():
            margin = landscape.place_shrubs(self.hf, band & allowed, recipe, rng)
            entry = density.setdefault("wash_margin", {"area_m2": 0.0, "count": 0, "recipe_per_ha": recipe.per_ha})
            entry["area_m2"] = round(entry["area_m2"] + float(np.count_nonzero(band & allowed)) * self.hf.res ** 2, 1)
            entry["count"] += len(margin)
            placed += margin
        self.sheet["shrub_density"] = {key: dict(entry, per_ha=round(entry["count"] / entry["area_m2"] * 1e4, 1))
                                       for key, entry in density.items()}
        meshed = self._inside(within if shrubs_3d is None else shrubs_3d, [(p.x, p.y) for p in placed])
        self.shrubs([(p.x, p.y, p.size, p.height) for p, m in zip(placed, meshed) if m])
        self.shrub_dots([(p.x, p.y, p.size) for p, m in zip(placed, meshed) if not m])

    def clutter_report(self, placements, allowed, sizes=True):
        """{type key: entry} for placements (landscape.Placement) on the
        ground raster: the area [m2] of each type where they were allowed
        (a boolean grid), their count and, with sizes, slabs' cumulative
        counts N(>=1, 2, 4 m) per 100 m2 and the cover by 1-7 m slabs (design
        5.5: the measured block fields' size-frequency)."""
        if not placements:
            return {}
        raster = self.ground_map()
        under = raster[self._samples([(p.x, p.y) for p in placements])]
        size = np.array([p.size for p in placements])
        out = {}
        for i in np.unique(under):
            area = float(np.count_nonzero((raster == i) & allowed)) * self.hf.res ** 2
            d = size[under == i]
            entry = {"area_m2": round(area, 1), "count": int(len(d))}
            if sizes:
                entry["per_100m2"] = {str(k): round(float(np.count_nonzero(d >= k)) / area * 100, 4) for k in (1, 2, 4)}
                big = d[(d >= 1.0) & (d < 7.0)]
                entry["cover_1_7"] = round(float(np.sum(math.pi / 4 * big * big)) / area, 4)
            out[self.legend[int(i)].key] = entry
        return out

    def _samples(self, points):
        """(rows, columns) of the heightmap samples nearest layout points (clamped to the grid)."""
        p = np.asarray(points, float).reshape(-1, 2)
        hf = self.hf
        col = np.round((p[:, 0] - hf.center[0] + hf.size / 2) / hf.res).astype(int)
        row = np.round((hf.center[1] + hf.size / 2 - p[:, 1]) / hf.res).astype(int)
        return np.clip(row, 0, hf.n - 1), np.clip(col, 0, hf.n - 1)

    def _inside(self, grid, points):
        """Whether each layout point lies on a True sample of a boolean grid
        on the heightmap's samples (nearest sample; every point for None)."""
        if grid is None or not len(points):
            return np.ones(len(points), bool)
        return np.asarray(grid, bool)[self._samples(points)]

    def imaged_shrubs(self, detected, within, rng_seed=4):
        """Shrubs found in a world's imagery (appearance.detect_shrubs, here
        in layout (x, y, spot diameter)) as meshes where `within` (a boolean
        grid) holds; the rest stay in the orthophoto as they are. A spot's
        diameter includes its shadow and blur: the crown is IMAGED_CROWN of
        it, 0.3-1.4 m, and the height that of the wash-margin recipe on a
        wash's margin (wash_margin), else the shrub recipe's of the ground
        under it (default the sand sheet's, lidar: at most 0.3 m on the
        plain)."""
        rng = np.random.default_rng([self.seed, rng_seed])
        meshed = [s for s, m in zip(detected, self._inside(within, [(x, y) for x, y, _ in detected])) if m]
        samples = self._samples([(x, y) for x, y, _ in meshed]) if meshed else (np.zeros(0, int),) * 2
        under = self.ground_map()[samples]
        margin = [None] * len(meshed)
        for band, recipe in self._in_margins():
            margin = [m or (recipe if inside else None) for m, inside in zip(margin, band[samples])]
        out = []
        for (x, y, d), index, beside in zip(meshed, under, margin):
            recipe = beside or self.legend[int(index)].clutter.shrubs or terrains.SAND_SHRUBS
            crown = float(np.clip(d * rng.uniform(*IMAGED_CROWN), 0.3, 1.4))
            out.append((x, y, crown, float(rng.uniform(*recipe.height_m))))
        self.shrubs(out)
        self.sheet["imaged_shrubs"] = {"detected": len(detected), "meshed": len(out)}

    def rock_field(self, name, rocks, colliding_size=0.1, palette="desert"):
        """Rocks on the terrain, counted in the sheet as group `name`. rocks:
        [(x, y, size [m], yaw)] in layout coordinates (from scatter,
        scatter_each or landscape.place); size is the long half-axis (about
        the rock's height); rocks smaller than colliding_size are visual
        only; each takes a colour of the terrains.ROCKS palette. Rocks grip
        like rock (ground.json's prefixes), wherever they lie.

        write() merges every group's rocks into meshes in the terrain's own
        link, one per CHUNK square (and colour, for the visuals), as separate
        rocks would cost more than the rover (see the module notes).
        Collisions use a coarser mesh of each rock's surface
        (meshes.hull_faces)."""
        colors = terrains.ROCKS[palette]
        colliding = 0
        for x, y, size, yaw in rocks:
            variant = int(self.rng.integers(meshes.ROCK_VARIANTS))
            color = tuple(colors[int(self.rng.integers(len(colors)))])
            R, z = self._rock_pose(x, y, size, variant, yaw)
            collides = size >= colliding_size
            colliding += collides
            self._rocks.append((self.to_world(x, y, z), size, variant, R, color, collides))
        group = self.sheet.setdefault("rocks", {}).setdefault(name, {"count": 0, "colliding": 0})
        group["count"] += len(rocks)
        group["colliding"] += colliding

    def _rock_pose(self, x, y, size, variant, yaw):
        """Orientation (3x3) and layout z of a rock at layout (x, y): its flat
        base laid on the plane of the ground under it, then sunk until every
        base vertex is ROCK_BURY * size below the ground, so that no edge of
        it floats on a slope."""
        R, z = self._lay(meshes.rock_base(variant) * size, x, y, yaw)
        return R, z - ROCK_BURY * size

    def _lay(self, base, x, y, yaw, tilt=0.0):
        """Orientation (3x3) and layout z of a shape whose base vertices
        `base` (its own frame, origin at the base) go at layout (x, y): yawed,
        tilted `tilt` [rad] about its own x axis, laid on the plane of the
        ground under it, as low as keeps every base vertex at or under the
        ground: the one standing highest above the ground touches it, so no
        edge floats (and a tilt buries the far edge)."""
        Rz = np.array(sdf.rpy_to_matrix(tilt, 0.0, yaw))
        P = base @ Rz.T
        A = np.column_stack([P[:, :2], np.ones(len(P))])
        (gx, gy, _), *_ = np.linalg.lstsq(A, self.height(x + P[:, 0], y + P[:, 1]), rcond=None)
        R = meshes.tilt(gx, gy) @ Rz
        P = base @ R.T
        return R, float(np.min(self.height(x + P[:, 0], y + P[:, 1]) - P[:, 2]))

    def slabs(self, name, placements):
        """Tabular blocks (design 5.5): landscape.Placement of slabs (size: the
        equivalent diameter D, height: the thickness) as meshes.slab
        variants, tilted on the ground under them (_lay) and sunk SLAB_BURY
        of their height; those SLAB_COLLIDE wide and wider collide. A tilt
        buries the slab's far edge by its span x sin(tilt), so it is capped
        where the top would go under there (slab_tilt). Colours: VARNISHED of
        them the varnish palette, the rest fresh sandstone. Counted in the
        sheet's slabs[name]."""
        colliding = 0
        for p in placements:
            variant = int(self.rng.integers(meshes.SLAB_VARIANTS))
            V, F = meshes.slab(variant)
            scale = np.array([p.size, p.size, p.height / meshes.SLAB_HEIGHT])
            base = V[V[:, 2] <= 1e-9] * scale
            R, z = self._lay(base, p.x, p.y, p.yaw, slab_tilt(p.tilt, np.ptp(base[:, 1]), p.height))
            palette = terrains.ROCKS["varnish" if self.rng.uniform() < terrains.VARNISHED else "fresh_sandstone"]
            color = tuple(palette[int(self.rng.integers(len(palette)))])
            xyz = np.array(self.to_world(p.x, p.y, z - SLAB_BURY * p.height))
            collides = p.size >= SLAB_COLLIDE
            colliding += collides
            self._pieces["slabs"].append(Piece((V * scale) @ R.T + xyz, F, F if collides else None, color))
        group = self.sheet.setdefault("slabs", {}).setdefault(name, {"count": 0, "colliding": 0})
        group["count"] += len(placements)
        group["colliding"] += colliding

    def risers(self, name, risers):
        """Sub-metre ledges along contours (design 5.5): landscape.Riser as
        meshes.riser_strip on the ground, the face downhill and the back
        buried in the ground behind it; they collide. Counted in the sheet's
        risers[name] with their total length."""
        length = 0.0
        for r in risers:
            path = np.asarray(r.path, float)
            if len(path) < 2:
                continue
            d = np.diff(path, axis=0)
            left = np.stack([-d[:, 1], d[:, 0]], axis=1) / np.maximum(np.linalg.norm(d, axis=1), 1e-9)[:, None]
            mid = (path[1:] + path[:-1]) / 2
            if np.mean(self.height(*(mid + left).T) - self.height(*(mid - left).T)) < 0:
                path = path[::-1]  # the ground must rise on the left: the face looks downhill
            z = self.height(path[:, 0], path[:, 1])
            world = np.column_stack([path - self.shift[:2], z - self.shift[2]])
            V, F = meshes.riser_strip(world, r.height, r.depth, self._world_height,
                                      seed=int(self.rng.integers(1 << 30)))
            palette = terrains.ROCKS["varnish" if self.rng.uniform() < terrains.VARNISHED else "fresh_sandstone"]
            self._pieces["risers"].append(Piece(V, F, F, tuple(palette[int(self.rng.integers(len(palette)))])))
            length += float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
        group = self.sheet.setdefault("risers", {}).setdefault(name, {"count": 0, "length_m": 0.0})
        group["count"] += len(risers)
        group["length_m"] = round(group["length_m"] + length, 1)

    def _world_height(self, x, y):
        """height() in the world frame: the visual surface at world (x, y)."""
        return self.height(x + self.shift[0], y + self.shift[1]) - self.shift[2]

    def shrubs(self, shrubs):
        """Desert shrubs: visual only (the rover drives through brush), each a
        meshes.shrub_lowpoly variant in a terrains.SHRUBS colour, merged by
        write() like the rocks (a model each cost ~3 us per step), standing
        SHRUB_SINK into the visual surface (height(): the collision surface
        lies the sinkage below it). shrubs: layout (x, y), or (x, y,
        diameter, height) [m] (default terrains.MISSION_SHRUB). The sheet
        counts them."""
        for s in shrubs:
            x, y, d, h = s if len(s) == 4 else (*s, *terrains.MISSION_SHRUB)
            V, F = meshes.shrub_lowpoly(int(self.rng.integers(meshes.SHRUB_VARIANTS)))
            R = np.array(sdf.rpy_to_matrix(0.0, 0.0, self.rng.uniform(0, 2 * math.pi)))
            color = tuple(terrains.SHRUBS[int(self.rng.integers(len(terrains.SHRUBS)))])
            xyz = np.array(self.to_world(x, y, self.height(x, y) - SHRUB_SINK))
            self._pieces["shrubs"].append(Piece((V * [d, d, h]) @ R.T + xyz, F, None, color))
            self._shrub_discs.append((x, y, d))
        self.sheet["shrubs"] = len(self._pieces["shrubs"])

    def shrub_dots(self, dots):
        """Shrubs too far out for meshes, drawn into the colour map as dark
        dots [(x, y, diameter)] (layout; design D10)."""
        self._dots += [(*self.to_world(x, y, 0.0)[:2], d) for x, y, d in dots]
        self.sheet["shrub_dots"] = len(self._dots)

    def pebbles(self, spots, density=terrains.PEBBLES, budget=terrains.PEBBLE_BUDGET):
        """Visual-only pebbles (design 5.5) in the discs spots [(x, y,
        radius)] (layout: round starts and targets): per square metre the
        recipe's (count, median diameter) pairs, log-normal sizes, each a
        meshes.pebble variant in a terrains.PEBBLE_COLOURS colour, sunk
        PEBBLE_SINK into the visual surface (height(): on the carved collision
        surface 87-94 % of them lay under the drawn sand, measured); thinned
        evenly when the discs would hold more than `budget` (+130 MB per
        20,000, M: render prototype)."""
        rng = np.random.default_rng([self.seed, 3])
        area = self.near(spots)
        expected = sum(count for count, _ in density) * area.sum() * self.hf.res ** 2
        thin = min(1.0, budget / max(expected, 1.0))
        cells = np.flatnonzero(area)
        placed = 0
        for count, d50 in density:
            k = int(rng.poisson(count * thin * len(cells) * self.hf.res ** 2)) if len(cells) else 0
            picked = cells[rng.integers(0, len(cells), k)] if k else np.zeros(0, int)
            rows, cols = np.divmod(picked, self.hf.n)
            x = self.hf.center[0] - self.hf.size / 2 + (cols + rng.uniform(-0.5, 0.5, k)) * self.hf.res
            y = self.hf.center[1] + self.hf.size / 2 - (rows + rng.uniform(-0.5, 0.5, k)) * self.hf.res
            d = d50 * np.exp(rng.normal(0, 0.5, k))
            z = self.height(x, y)
            for i in range(k):
                V, F = meshes.pebble(int(rng.integers(meshes.PEBBLE_VARIANTS)), 1 if d[i] > 0.04 else 0)
                scale = d[i] * np.array([rng.uniform(0.7, 1.3), rng.uniform(0.7, 1.3), rng.uniform(0.5, 1.0)])
                R = np.array(sdf.rpy_to_matrix(0.0, 0.0, rng.uniform(0, 2 * math.pi)))
                xyz = np.array(self.to_world(x[i], y[i], z[i] - PEBBLE_SINK * 0.55 * scale[2]))
                color = tuple(terrains.PEBBLE_COLOURS[int(rng.integers(len(terrains.PEBBLE_COLOURS)))])
                self._pieces["pebbles"].append(Piece((V * scale) @ R.T + xyz, F, None, color))
            placed += k
        self.sheet["pebbles"] = {"count": self.sheet.get("pebbles", {}).get("count", 0) + placed,
                                 "thinned_to": round(thin, 3)}

    def _write_clutter(self):
        """All clutter merged by kind into one mesh per CHUNK square of the
        world in the terrain's link: visual <kind>_<i>_<j> (binary glTF, a
        primitive per colour), collision <kind>_<i>_<j> (OBJ) for the kinds
        that collide."""
        for xyz, size, variant, R, color, collides in self._rocks:
            # The rock's own 162 vertices for boulders, 42 of them for the rest.
            visual = meshes.hull_faces(2 if size >= 0.5 else 1)
            collision = meshes.hull_faces(1 if size >= 0.2 else 0) if collides else None
            V = meshes.rock_variant(variant)[0][:visual.max() + 1] * size @ R.T + xyz
            self._pieces["rocks"].append(Piece(V, visual, collision, color))
        model, _, link = self._terrain
        directory = self.models_dir / model / "meshes"
        for kind in sorted(self._pieces):
            chunks = defaultdict(list)
            for piece in self._pieces[kind]:
                cx, cy = piece.V[:, :2].mean(axis=0)
                chunks[math.floor(cx / CHUNK), math.floor(cy / CHUNK)].append(piece)
            roughness, shadows = STYLE[kind]
            for (i, j), pieces in sorted(chunks.items()):
                name = f"{kind}_{i}_{j}"
                origin = np.array([i * CHUNK, j * CHUNK, 0.0])
                by_colour = defaultdict(list)
                for piece in pieces:
                    by_colour[piece.color].append((piece.V - origin, piece.visual))
                parts = [(*meshes.combine(by_colour[c]), None, None, meshes.Material(c, roughness))
                         for c in sorted(by_colour)]
                meshes.write_glb_parts(directory / f"{name}.glb", parts)
                sdf.visual(link, name, sdf.mesh(sdf.model_uri(model, "meshes", f"{name}.glb")), tuple(origin),
                           color=None, cast_shadows=shadows)
                solid = [(p.V - origin, p.collision) for p in pieces if p.collision is not None]
                if solid:
                    V, F = meshes.combine(solid)
                    used = np.unique(F)  # rocks' collision faces use the first of their vertices only
                    remap = np.zeros(len(V), int)
                    remap[used] = np.arange(len(used))
                    meshes.write_obj(directory / f"{name}.obj", V[used], remap[F])
                    sdf.collision(link, name, sdf.mesh(sdf.model_uri(model, "meshes", f"{name}.obj")), tuple(origin))
        self.sheet["clutter_triangles"] = {kind: int(sum(len(p.visual) for p in pieces))
                                           for kind, pieces in sorted(self._pieces.items())}

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

    # --- Output ----------------------------------------------------------------------

    def write(self):
        self._write_terrain()
        self._write_clutter()
        self._write_ground()
        if self._look is not None:
            self._write_colour()
            self._write_farfield()
        name, root, _ = self._terrain
        for collision in self._terrain[2].findall("collision"):
            sdf.collide_bitmask(collision, sdf.GROUND)
        sdf.write_model(self.models_dir, name, root, f"Terrain for the URC {self.sheet['mission']} world.")
        self.worlds_dir.mkdir(parents=True, exist_ok=True)
        world_path = self.worlds_dir / f"{self.name}.sdf"
        world_path.write_text(sdf.document(self.root))
        sheet_path = self.worlds_dir / f"{self.name}.json"
        sheet_path.write_text(json.dumps(self.sheet, indent=2) + "\n")
        return world_path, sheet_path
