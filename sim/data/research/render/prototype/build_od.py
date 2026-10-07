import json, sys, numpy as np
from PIL import Image
import detail_tex as dt
from terra_mix import compensate, modulation, smoothstep
from variant import make_variant
T='/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/vis.jRJ3'

def strong_normals(names_px, strength):
    for name, size_m in names_px:
        h = np.asarray(Image.open(f'tex/detail/{name}_height.png')).astype(np.float32)/65535
        # height pngs are normalised: recover metres from the generator's typical relief
        Image.fromarray(dt.normal_from_height(h * RELIEF[name], size_m/h.shape[0], strength)).save(f'tex/detail/{name}_nrm{strength}.png')

RELIEF = {'pavement': 0.02, 'popcorn': 0.014, 'sand': 0.007, 'slab': 0.05}

def build(tag, layers, ortho='tex/ortho_4096.png', strength=3):
    """layers: [(name, tile_m, min_h, fade)]"""
    ortho = np.asarray(Image.open(ortho))
    H = np.load('tex/H4096.npy')
    mean = dt.srgb_to_lin(ortho).reshape(-1,3).mean(0)
    spec, tex = [], []
    for name, tile, mn, fade in layers:
        m = modulation(f'tex/detail/{name}_diffuse.png', mean, f'tex/detail/{name}_mod.png')
        spec.append((m, mn, fade))
        RELIEFS = RELIEF[name]
        h = np.asarray(Image.open(f'tex/detail/{name}_height.png')).astype(np.float32)/65535
        Image.fromarray(dt.normal_from_height(h*RELIEFS, tile/h.shape[0], strength)).save(f'tex/detail/{name}_nrm.png')
        tex.append((f'{T}/tex/detail/{name}_mod.png', f'{T}/tex/detail/{name}_nrm.png', tile))
    Op, clip = compensate(ortho, H, spec, 'linear')
    Image.fromarray(Op).save(f'tex/O_{tag}.png')
    print(tag, 'clipped', round(clip, 4))
    make_variant(tag, 'urc_terrain_autonomy', [(f'{T}/tex/O_{tag}.png', f'{T}/tex/flat_normal.png', 2048)] + tex,
                 [(mn, fade) for _, _, mn, fade in layers], 'worlds/urc_autonomy.sdf', f'worlds/v_{tag}.sdf')
    return f'worlds/v_{tag}.sdf'
