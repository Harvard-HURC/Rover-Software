#!/usr/bin/env python3
"""
Write the tread tyre's mesh, meshes/wheel.glb.

The tyre is a cylinder of radius 1 and width 1 round the y axis (the wheel link's axle; the
description scales it to the wheel), flat-shaded: a dark tread with light bars across it, and on
each face a light hub disc and spokes out to the rim, light but for the one at angle 0 (along the
link's x), which has the chassis' colour on both faces, so that a wheel spinning while the rover
stands still shows. Same outline as a plain cylinder: what changes is paint, not shape. The
colours are config/rover.yaml's looks.colors (tyre, tread, chassis).

    python3 rover_description/meshes/gen_wheel_mesh.py

rewrites meshes/wheel.glb; run it after changing those colours, then `pixi run sim-model`, which
copies the mesh into the simulation's rover model. Needs numpy and PyYAML.
"""
import json
import math
from pathlib import Path
import struct

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROVER_YAML = HERE.parent / 'config' / 'rover.yaml'
WHEEL_GLB = HERE / 'wheel.glb'
ROUGHNESS = 0.9
GENERATOR = 'rover_description/meshes/gen_wheel_mesh.py'


def colors(rover_yaml=ROVER_YAML):
    """Return the tread's (dark, light, mark) RGB colours from rover.yaml."""
    looks = yaml.safe_load(Path(rover_yaml).read_text())['looks']['colors']
    return tuple(tuple(looks[name][:3]) for name in ('tyre', 'tread', 'chassis'))


def wheel_parts(dark, light, mark, segments=48, bars=16, spokes=4, spoke_deg=15.0, hub=0.35):
    """Return the tyre as [(vertices, triangles, normals, colour)], one part per colour."""
    buckets = {colour: ([], []) for colour in (dark, light, mark)}  # triangles, their normals

    def tri(colour, a, b, c, n):
        a, b, c, n = (np.asarray(v, float) for v in (a, b, c, n))
        if np.dot(np.cross(b - a, c - a), n) < 0:  # counter-clockwise seen from outside
            b, c = c, b
        buckets[colour][0].append((a, b, c))
        buckets[colour][1].append((n, n, n))

    step = 360.0 / segments
    for k in range(segments):
        a0, a1 = math.radians(k * step), math.radians((k + 1) * step)
        mid = (k + 0.5) * step
        tread = light if k % (segments // bars) == 0 else dark
        n = (math.cos(math.radians(mid)), 0.0, math.sin(math.radians(mid)))
        rim = [(math.cos(a), y, math.sin(a))
               for a, y in ((a0, -0.5), (a1, -0.5), (a1, 0.5), (a0, 0.5))]
        tri(tread, rim[0], rim[1], rim[2], n)
        tri(tread, rim[0], rim[2], rim[3], n)
        off = min(mid % (360.0 / spokes), 360.0 / spokes - mid % (360.0 / spokes))  # from a spoke
        spoke = dark
        if off < spoke_deg / 2:
            spoke = mark if min(mid, 360.0 - mid) < spoke_deg / 2 else light
        for y in (-0.5, 0.5):
            face = (0.0, math.copysign(1.0, y), 0.0)

            def ring(r, a, y=y):
                return (r * math.cos(a), y, r * math.sin(a))

            tri(light, (0.0, y, 0.0), ring(hub, a0), ring(hub, a1), face)
            tri(spoke, ring(hub, a0), ring(1.0, a0), ring(1.0, a1), face)
            tri(spoke, ring(hub, a0), ring(1.0, a1), ring(hub, a1), face)
    parts = []
    for colour, (triangles, normals) in buckets.items():
        vertices = np.array(triangles).reshape(-1, 3)
        parts.append((vertices, np.arange(len(vertices)).reshape(-1, 3),
                      np.array(normals).reshape(-1, 3), colour))
    return parts


def write_glb(path, parts):
    """Write the parts as one binary glTF mesh, a primitive and a material per part."""
    blob = bytearray()
    views, accessors, primitives, materials = [], [], [], []

    def view(data, target):
        blob.extend(b'\0' * (-len(blob) % 4))
        views.append({'buffer': 0, 'byteOffset': len(blob), 'byteLength': len(data),
                      'target': target})
        blob.extend(data)
        return len(views) - 1

    def accessor(array, kind, component, target, bounds=False):
        entry = {'bufferView': view(array.tobytes(), target), 'componentType': component,
                 'count': int(len(array)), 'type': kind}
        if bounds:
            entry['min'] = [float(v) for v in array.min(axis=0)]
            entry['max'] = [float(v) for v in array.max(axis=0)]
        accessors.append(entry)
        return len(accessors) - 1

    for vertices, triangles, normals, colour in parts:
        attributes = {
            'POSITION': accessor(vertices.astype(np.float32), 'VEC3', 5126, 34962, bounds=True),
            'NORMAL': accessor(normals.astype(np.float32), 'VEC3', 5126, 34962)}
        indices = accessor(triangles.astype(np.uint32).reshape(-1), 'SCALAR', 5125, 34963)
        primitives.append({'attributes': attributes, 'indices': indices,
                           'material': len(materials)})
        materials.append({'pbrMetallicRoughness': {
            'baseColorFactor': [float(c) for c in colour] + [1.0], 'metallicFactor': 0.0,
            'roughnessFactor': ROUGHNESS}})
    gltf = {'asset': {'version': '2.0', 'generator': GENERATOR}, 'scene': 0,
            'scenes': [{'nodes': [0]}], 'nodes': [{'mesh': 0}],
            'meshes': [{'primitives': primitives}], 'materials': materials,
            'buffers': [{'byteLength': len(blob)}], 'bufferViews': views, 'accessors': accessors}
    text = json.dumps(gltf, separators=(',', ':')).encode()
    text += b' ' * (-len(text) % 4)
    blob.extend(b'\0' * (-len(blob) % 4))
    with open(path, 'wb') as f:
        f.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(text) + 8 + len(blob)))  # glTF 2
        f.write(struct.pack('<II', len(text), 0x4E4F534A) + text)  # chunk JSON
        f.write(struct.pack('<II', len(blob), 0x004E4942) + bytes(blob))  # chunk BIN


def main():
    write_glb(WHEEL_GLB, wheel_parts(*colors()))
    print(f'wrote {WHEEL_GLB}')


if __name__ == '__main__':
    main()
