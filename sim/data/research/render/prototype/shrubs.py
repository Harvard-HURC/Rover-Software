import math, numpy as np
from pathlib import Path
import sys
sys.path.insert(0, "/Users/alarion239/Desktop/Rover/sim")
from urc import meshes
COLS=[(0.36,0.36,0.29),(0.30,0.31,0.24),(0.42,0.40,0.32),(0.33,0.30,0.24),(0.27,0.29,0.22)]
def shrub_field(out_obj, xy_diam, hz, center, radius, seed=0):
    rng=np.random.default_rng(seed)
    V0,F0=meshes.icosphere(1)
    sel=xy_diam[np.hypot(xy_diam[:,0]-center[0], xy_diam[:,1]-center[1])<radius]
    parts={k:([],[]) for k in range(len(COLS))}
    for x,y,dm in sel:
        crown=np.clip(dm*rng.uniform(0.45,0.7),0.3,1.4)   # NAIP blobs include shadow and blur
        height=crown*rng.uniform(0.45,0.8)
        z=hz(x,y)
        k=int(rng.integers(len(COLS)))
        for b in range(int(rng.integers(4,8))):
            r=crown/2*rng.uniform(0.35,0.6)
            ox,oy=rng.normal(0,crown/5,2)
            oz=rng.uniform(0.3,0.9)*height-r*0.3
            jit=1+0.25*rng.uniform(-1,1,(len(V0),1))
            P=V0*jit*[r,r,r*rng.uniform(0.7,1.0)]+[x+ox,y+oy,z+max(oz,r*0.4)]
            parts[k][0].append(P); parts[k][1].append(F0)
    lines=[f"mtllib {Path(out_obj).stem}.mtl"]; mtl=[]; vb=0; tris=0
    for k,(Ps,Fs) in parts.items():
        if not Ps: continue
        V=np.concatenate(Ps); offs=np.cumsum([0]+[len(p) for p in Ps[:-1]])
        F=np.concatenate([f+o for f,o in zip(Fs,offs)]); N=meshes.vertex_normals(V,F)
        r,g,b=COLS[k]; mtl.append(f"newmtl s{k}\nKd {r} {g} {b}\nKa {r} {g} {b}\nKs 0.02 0.02 0.02\nNs 5\n")
        lines.append(f"o shrubs{k}\nusemtl s{k}")
        lines+= [f"v {a:.3f} {b_:.3f} {c:.3f}" for a,b_,c in V]
        lines+= [f"vn {a:.3f} {b_:.3f} {c:.3f}" for a,b_,c in N]
        lines+= [f"f {a+vb}//{a+vb} {b_+vb}//{b_+vb} {c+vb}//{c+vb}" for a,b_,c in F+1]
        vb+=len(V); tris+=len(F)
    Path(out_obj).write_text("\n".join(lines)+"\n"); Path(out_obj).with_suffix('.mtl').write_text("\n".join(mtl))
    return len(sel), tris
