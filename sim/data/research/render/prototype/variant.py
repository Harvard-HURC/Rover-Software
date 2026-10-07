"""Terrain-visual variants of a URC world, for render comparisons.

make_variant(tag, base_model, layers, blends, world_in, world_out, extra_files)
layers: [(diffuse path, normal path or None, size)], blends: [(min_height, fade)].
Creates models/<base_model>_<tag> (model.sdf with the terrain visual replaced; textures copied)
and a world whose terrain include points at it.
"""
import re
import shutil
from pathlib import Path

HERE = Path(__file__).resolve().parent
MODELS = HERE / "models"


def heightmap_visual(name, base_model, layers, blends, size_xyz):
    out = ["<heightmap>", "<use_terrain_paging>false</use_terrain_paging>"]
    for diffuse, normal, size in layers:
        out.append("<texture>")
        out.append(f"<diffuse>model://{name}/textures/{Path(diffuse).name}</diffuse>")
        if normal:
            out.append(f"<normal>model://{name}/textures/{Path(normal).name}</normal>")
        else:
            out.append("<normal>no_normal_map.png</normal>")
        out.append(f"<size>{size}</size>")
        out.append("</texture>")
    for min_height, fade in blends:
        out.append(f"<blend><min_height>{min_height}</min_height><fade_dist>{fade}</fade_dist></blend>")
    out.append(f"<uri>model://{base_model}/heightmap.png</uri>")
    out.append(f"<size>{size_xyz}</size>")
    out.append("</heightmap>")
    return "\n".join(out)


def make_variant(tag, base_model, layers, blends, world_in, world_out, model_sub=(), drop_rocks=False):
    name = f"{base_model}_{tag}"
    d = MODELS / name
    shutil.rmtree(d, ignore_errors=True)
    (d / "textures").mkdir(parents=True)
    for diffuse, normal, _ in layers:
        for p in (diffuse, normal):
            if p:
                dst = d / "textures" / Path(p).name
                if not dst.exists():
                    dst.symlink_to(Path(p).resolve())
    src = (MODELS / base_model / "model.sdf").read_text()
    size_xyz = re.search(r'<visual name="terrain_visual">.*?<size>([^<]*)</size>\s*</heightmap>', src, re.S).group(1)
    new_vis = heightmap_visual(name, base_model, layers, blends, size_xyz)
    src = re.sub(r'(<visual name="terrain_visual">.*?)<heightmap>.*?</heightmap>', lambda m: m.group(1) + new_vis,
                 src, count=1, flags=re.S)
    src = src.replace(f'<model name="{base_model}">', f'<model name="{name}">')
    # Relative model:// references to the base model's own files keep working.
    for pattern, repl in model_sub:
        src = re.sub(pattern, repl, src, flags=re.S)
    (d / "model.sdf").write_text(src)
    (d / "model.config").write_text((MODELS / base_model / "model.config").read_text().replace(base_model, name))
    w = Path(world_in).read_text().replace(f"model://{base_model}</uri>", f"model://{name}</uri>")
    Path(world_out).write_text(w)
    return name
