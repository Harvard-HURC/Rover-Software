"""Shared assets: the textures and meshes every model and world uses, in
models_dir/urc_media (Media)."""
import functools
import inspect
import os
import sys
import types
import zlib
from pathlib import Path

from . import meshes, sdf, textures


class Media:
    """Textures and meshes shared by every model and world: models_dir/urc_media.

    A file is named by its stem (what it is: type, variant) plus a hash of
    what makes it (_digest): the generator, its arguments with their
    defaults, the source of its module and of every module of this package
    that module uses, and the contents of the files its arguments name. So
    each file is made once and skipped while it is current, on later runs
    too, and any change makes a new file under a new name; gen_worlds.py
    prunes the files no model uses any more.

    Kinds: PNG textures in textures/, OBJ meshes and GLB meshes (glb()) in
    meshes/, so prune(), which walks the SUFFIX directories, handles all."""

    NAME = "urc_media"
    SUFFIX = {"textures": ".png", "meshes": ".obj"}  # directory -> its default file type

    def __init__(self, models_dir):
        self.models_dir = Path(models_dir)
        self.dir = self.models_dir / self.NAME
        for kind in self.SUFFIX:
            (self.dir / kind).mkdir(parents=True, exist_ok=True)
        root, _ = sdf.model_root(self.NAME, static=True)
        sdf.write_model(self.models_dir, self.NAME, root, "Shared URC textures and meshes (no geometry).")

    def _uri(self, kind, stem, make, args, kwargs, suffix=None):
        filename = f"{stem}_{_digest(make, args, kwargs)}{suffix or self.SUFFIX[kind]}"
        path = self.dir / kind / filename
        if not path.exists():
            # Made under a hidden name of this process's, then renamed: never a
            # half-written file under the real name, and overlapping runs
            # (gen_worlds.lock) making the same file do not write one file.
            partial = path.with_name(f".{path.stem}.{os.getpid()}{path.suffix}")
            make(partial, *args, **kwargs)
            partial.replace(path)
        return sdf.model_uri(self.NAME, kind, filename)

    def texture(self, stem, make, *args, **kwargs):
        """URI of the PNG that make(path, *args, **kwargs) writes."""
        return self._uri("textures", stem, make, args, kwargs)

    def mesh(self, stem, make, *args, **kwargs):
        """URI of the OBJ that make(path, *args, **kwargs) writes."""
        return self._uri("meshes", stem, make, args, kwargs)

    def glb(self, stem, make, *args, **kwargs):
        """URI of the binary glTF mesh that make(path, *args, **kwargs) writes."""
        return self._uri("meshes", stem, make, args, kwargs, suffix=".glb")

    def path(self, uri):
        """The file of one of these URIs (model://urc_media/<kind>/<file>)."""
        prefix = sdf.model_uri(self.NAME) + "/"
        if not uri.startswith(prefix):
            raise ValueError(f"{uri} is not in {self.NAME}")
        return self.dir / uri[len(prefix):]

    def aruco(self, tag_id):
        return self.texture(f"aruco_4x4_50_{tag_id}", textures.write_aruco, tag_id)

    def sign(self, name, lines, **style):
        return self.texture(f"sign_{name}", textures.sign_image, lines, **style)

    def quad(self):
        return self.mesh("quad", meshes.write_quad)

    def detail(self, key, mean=None):
        """URIs (diffuse, normal) of the ground detail texture
        textures.DETAILS[key]; mean: the linear RGB its diffuse map is
        rescaled to (textures.detail_texture), rounded to 1/1000 so that one
        colour map always names the same file."""
        mean = None if mean is None else tuple(round(float(m), 3) for m in mean)
        return (self.texture(f"detail_{key}_diffuse", textures.detail_texture, key, "diffuse", mean=mean),
                self.texture(f"detail_{key}_normal", textures.detail_texture, key, "normal"))

    def flat_normal(self):
        return self.texture("flat_normal", textures.flat_normal)

    def dust_puff(self, rgb, opacity):
        return self.texture("dust_puff", textures.dust_puff, tuple(rgb), opacity)


@functools.lru_cache(maxsize=None)
def _source_crc(module):
    """CRC of a module's source and, in order, of every module of its own
    package it imports, directly or through them (farfield's texture uses
    appearance's Boost and textures' colour conversions)."""
    package = module.rpartition(".")[0]
    seen, todo = set(), [module]
    while todo:
        name = todo.pop()
        if name in seen:
            continue
        seen.add(name)
        todo += [v.__name__ for v in vars(sys.modules[name]).values()
                 if package and isinstance(v, types.ModuleType) and v.__name__.rpartition(".")[0] == package]
    crc = 0
    for name in sorted(seen):
        crc = zlib.crc32(Path(sys.modules[name].__file__).read_bytes(), crc)
    return crc


@functools.lru_cache(maxsize=None)
def _file_crc(path, size, mtime_ns):
    """CRC of a file's contents (cached while its size and time stay)."""
    return zlib.crc32(Path(path).read_bytes())


def _digest(make, args, kwargs):
    """Hash of a generator, its arguments (defaults included), its sources
    (_source_crc) and the files its arguments name (a raster read by the
    generator, say)."""
    bound = inspect.signature(make).bind(None, *args, **kwargs)  # the first parameter is the output path
    bound.apply_defaults()
    values = list(bound.arguments.items())[1:]
    crc = zlib.crc32(repr((make.__qualname__, values)).encode(), _source_crc(make.__module__))
    for _, value in values:
        if isinstance(value, (str, Path)) and os.path.isfile(value):
            stat = os.stat(value)
            crc = zlib.crc32(_file_crc(str(value), stat.st_size, stat.st_mtime_ns).to_bytes(4, "little"), crc)
    return f"{crc:08x}"
