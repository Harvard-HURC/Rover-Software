"""Render-throughput benchmark: a world with only visuals (no collisions, no rover), one camera at a
very high rate; frames delivered per wall-second ~ render throughput."""
import json, re, shutil
from pathlib import Path
from go import render
T = Path('/private/tmp/claude-502/-Users-alarion239-Desktop-Rover/d048031d-a613-49e0-ad24-8cb3b1a40ce3/scratchpad/vis.jRJ3')

def visual_only_model(src_model, dst_model):
    d = T / 'models' / dst_model
    shutil.rmtree(d, ignore_errors=True)
    shutil.copytree(T / 'models' / src_model, d, symlinks=True)
    s = (d / 'model.sdf').read_text()
    s = re.sub(r'<collision name=.*?</collision>', '', s, flags=re.S)
    s = s.replace(f'<model name="{src_model}">', f'<model name="{dst_model}">')
    (d / 'model.sdf').write_text(s)
    c = (d / 'model.config').read_text().replace(src_model, dst_model)
    (d / 'model.config').write_text(c)

def world(src_world, terrain_model, out, extra=''):
    w = (T / src_world).read_text()
    head = w[:w.index('<include>')]
    head = re.sub(r'<plugin filename="gz-sim-(imu|navsat|user-commands)-system".*?/>', '', head, flags=re.S)
    body = f'<include><uri>model://{terrain_model}</uri><name>terrain</name><pose>0 0 0 0 0 0</pose></include>{extra}</world></sdf>'
    (T / out).write_text(head + body)
    return out

def run(tag, world_path, views, seconds=2.0, env=None, replace=(), extra=''):
    r = render(tag, world_path, views=views, seconds=seconds, env=env, replace=replace, extra_world=extra)
    f = sum(r['frames'].values())
    res = dict(tag=tag, frames=f, run_s=r['run_s'], fps=round(f / r['run_s'], 1), ms_per_frame=round(r['run_s'] / max(f, 1) * 1000, 2),
               peak_MB=r['peak_footprint_MB'], errors=r['errors'][:2])
    print(json.dumps(res), flush=True)
    return res
