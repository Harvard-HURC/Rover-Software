"""Shared assets: the textures and meshes every model and world uses, in
models_dir/urc_media (Media)."""
import functools
import os
import sys
import zlib
from pathlib import Path

from . import meshes, sdf, textures


class Media:
    """Textures and meshes shared by every model and world: models_dir/urc_media.

    A file is named by its stem (what it is: type, variant) plus a hash of
    what makes it: the generator, its arguments and its module's source. So
    each file is made once and skipped while it is current, on later runs
    too, and any change makes a new file under a new name; gen_worlds.py
    prunes the files no model uses any more."""

    NAME = "urc_media"
    SUFFIX = {"textures": ".png", "meshes": ".obj"}

    def __init__(self, models_dir):
        self.models_dir = Path(models_dir)
        self.dir = self.models_dir / self.NAME
        for kind in self.SUFFIX:
            (self.dir / kind).mkdir(parents=True, exist_ok=True)
        root, _ = sdf.model_root(self.NAME, static=True)
        sdf.write_model(self.models_dir, self.NAME, root, "Shared URC textures and meshes (no geometry).")

    def _uri(self, kind, stem, make, args, kwargs):
        filename = f"{stem}_{_digest(make, args, kwargs)}{self.SUFFIX[kind]}"
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

    def aruco(self, tag_id):
        return self.texture(f"aruco_4x4_50_{tag_id}", textures.write_aruco, tag_id)

    def sign(self, name, lines, **style):
        return self.texture(f"sign_{name}", textures.sign_image, lines, **style)

    def quad(self):
        return self.mesh("quad", meshes.write_quad)


@functools.lru_cache(maxsize=None)
def _source_crc(module):
    return zlib.crc32(Path(sys.modules[module].__file__).read_bytes())


def _digest(make, args, kwargs):
    params = repr((make.__qualname__, args, sorted(kwargs.items())))
    return f"{zlib.crc32(params.encode(), _source_crc(make.__module__)):08x}"
