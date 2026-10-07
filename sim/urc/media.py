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

    def dust_puff(self):
        return self.texture("dust_puff", textures.dust_puff)


@functools.lru_cache(maxsize=None)
def _source_crc(module):
    return zlib.crc32(Path(sys.modules[module].__file__).read_bytes())


def _digest(make, args, kwargs):
    params = repr((make.__qualname__, args, sorted(kwargs.items())))
    return f"{zlib.crc32(params.encode(), _source_crc(make.__module__)):08x}"
