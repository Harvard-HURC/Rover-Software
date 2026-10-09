import json, re, sys
from atmos import write_atmosphere, lighting
from go import render
T='/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/vis.jRJ3'
PATCH={'GZ_RENDERING_RESOURCE_PATH': f'{T}/gzr'}
CAM=[{"name": "bench", "pose": [166, 121.22, 58.18, 0, 0.3, 1.5708], "size": [1280, 720], "hfov": 1.2, "rate": 30}]
ONB=[{"name": "bench", "pose": [166, 126.4, 56.78, 0, 0.12, 1.5708], "size": [1280, 720], "hfov": 1.5, "rate": 30}]
def bench(tag, world, env=None, extra='', replace=(), views=CAM, seconds=6.0):
    out = {}
    for cam in ('cam', 'nocam'):
        r = render(f'b_{tag}_{cam}', world, views=views if cam == 'cam' else [dict(views[0], rate=0.0001)], seconds=seconds,
                   env=env, extra_world=extra, replace=replace)
        out[cam] = r
    f = out['cam']['frames']['bench']
    ms = (out['cam']['run_s'] - out['nocam']['run_s']) / max(f, 1) * 1000
    res = dict(tag=tag, frames=f, run_cam=out['cam']['run_s'], run_nocam=out['nocam']['run_s'], ms_per_frame=round(ms, 2),
               peak_MB=out['cam']['peak_footprint_MB'], load_s=out['cam']['load_s'], errors=out['cam']['errors'][:3])
    print(json.dumps(res), flush=True)
    return res
