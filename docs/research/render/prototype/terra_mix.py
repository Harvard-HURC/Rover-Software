"""Ortho + detail on gz-sim's Terra heightmap within its limits.

gz-rendering's Terra lerps each detail layer over the one below it by a height weight:
    c = O'; for i: c = lerp(c, D_i, w_i(h)),   w_i(h) = smoothstep(min_i, min_i + fade_i, h)
so final = a0 * O' + sum_i a_i * D_i, with a0 = prod(1 - w_i) and a_i = w_i * prod_{j>i}(1 - w_j).
Pre-compensating the ortho per texel,
    O' = (O - sum_i a_i * m_i) / a0      (m_i = mean colour of D_i)
makes the result O + sum_i a_i (D_i - m_i): the orthophoto everywhere plus zero-mean detail.
"""
import numpy as np
from PIL import Image

from detail_tex import lin_to_srgb, srgb_to_lin

Image.MAX_IMAGE_PIXELS = None


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0, 1)
    return t * t * (3 - 2 * t)


def texel_heights(heightmap_png, z_max, n_tex):
    h = np.asarray(Image.open(heightmap_png)).astype(np.float32) / 65535 * z_max
    import cv2
    s = h.shape[0] - 1  # samples span the terrain edge to edge
    c = ((np.arange(n_tex) + 0.5) / n_tex * s).astype(np.float32)
    X, Y = np.meshgrid(c, c)
    return cv2.remap(h, X, Y, cv2.INTER_LINEAR)


def compensate(ortho_srgb, heights, layers, space="linear"):
    """layers: [(mean colour (linear, 3), min_height, fade)] bottom to top.
    Returns O' (sRGB uint8) and the fraction of texels clipped."""
    to = srgb_to_lin if space == "linear" else (lambda c: np.asarray(c, np.float32) / 255)
    back = lin_to_srgb if space == "linear" else (lambda c: np.round(np.clip(c, 0, 1) * 255).astype(np.uint8))
    O = to(ortho_srgb)
    a0 = np.ones(heights.shape, np.float32)
    acc = np.zeros(O.shape, np.float32)
    for mean, mn, fade in layers:
        w = smoothstep(mn, mn + fade, heights)[..., None]
        m = np.asarray(mean, np.float32) if space == "linear" else np.asarray(lin_to_srgb(mean), np.float32) / 255
        acc = acc * (1 - w) + w * m
        a0 = a0 * (1 - w[..., 0])
    Op = (O - acc) / a0[..., None]
    clipped = float(np.mean((Op < 0) | (Op > 1)))
    return back(Op), clipped


def modulation(diffuse_png, mean_lin, out_png):
    """Detail texture rescaled per channel (linear) so its mean is mean_lin."""
    D = srgb_to_lin(np.asarray(Image.open(diffuse_png).convert("RGB")))
    D = D * (np.asarray(mean_lin) / D.reshape(-1, 3).mean(axis=0))
    Image.fromarray(lin_to_srgb(D)).save(out_png)
    return D.reshape(-1, 3).mean(axis=0)
