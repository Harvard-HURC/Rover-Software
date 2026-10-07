"""Shrubs and pebbles as GLB (one file per colour) for lower load memory than OBJ."""
import math, sys, numpy as np
from pathlib import Path
sys.path.insert(0, "/Users/alarion239/Desktop/Rover/sim")
from urc import meshes
from glb import write_glb

def _emit(out_dir, prefix, parts, colors):
    files=[]; tris=0
    for k,(Ps,Fs) in parts.items():
        if not Ps: continue
        V=np.concatenate(Ps); offs=np.cumsum([0]+[len(p) for p in Ps[:-1]])
        F=np.concatenate([f+o for f,o in zip(Fs,offs)]); N=meshes.vertex_normals(V,F)
        f=Path(out_dir)/f'{prefix}{k}.glb'
        write_glb(f, V, N, np.zeros((len(V),2)), F, yup=False, color=colors[k], roughness=0.95)
        files.append(f.name); tris+=len(F)
    return files, tris

SHRUB_COLS=[(0.30,0.30,0.23),(0.25,0.26,0.19),(0.36,0.34,0.26),(0.28,0.25,0.19),(0.22,0.24,0.17)]
def shrubs(out_dir, xy_diam, hz, center, radius, seed=0, keep_clear=()):
    rng=np.random.default_rng(seed); V0,F0=meshes.icosphere(0)
    sel=xy_diam[np.hypot(xy_diam[:,0]-center[0], xy_diam[:,1]-center[1])<radius]
    for (cx,cy,cr) in keep_clear:
        sel=sel[np.hypot(sel[:,0]-cx, sel[:,1]-cy)>cr]
    parts={k:([],[]) for k in range(len(SHRUB_COLS))}
    for x,y,dm in sel:
        crown=np.clip(dm*rng.uniform(0.45,0.7),0.3,1.4); height=crown*rng.uniform(0.45,0.8); z=hz(x,y)
        k=int(rng.integers(len(SHRUB_COLS)))
        for b in range(int(rng.integers(6,11))):
            r=crown/2*rng.uniform(0.22,0.42)
            ox,oy=rng.normal(0,crown/4.5,2); oz=rng.uniform(0.2,1.0)*height
            jit=1+0.35*rng.uniform(-1,1,(len(V0),1))
            P=V0*jit*[r,r,r*rng.uniform(0.7,1.1)]+[x+ox,y+oy,z+max(oz,r*0.5)]
            parts[k][0].append(P); parts[k][1].append(F0)
    files,tris=_emit(out_dir,'shrubs',parts,SHRUB_COLS)
    return files,tris,len(sel)

PEB_COLS=[(0.42,0.33,0.27),(0.30,0.24,0.21),(0.55,0.47,0.40),(0.62,0.40,0.30),(0.20,0.17,0.16),(0.70,0.66,0.60)]
def pebbles(out_dir, hz, center, radius, density, seed=0, keep_out=1.2):
    rng=np.random.default_rng(seed)
    protos=[meshes.rock(s,(1.0,1.0,0.55),roughness=0.25,subdivisions=1,flat_bottom=0.3) for s in range(12)]
    protos0=[meshes.rock(s,(1.0,1.0,0.55),roughness=0.25,subdivisions=0,flat_bottom=0.3) for s in range(12)]
    parts={k:([],[]) for k in range(len(PEB_COLS))}; count=0
    for per_m2,d50 in density:
        n=rng.poisson(per_m2*math.pi*radius**2); r=radius*np.sqrt(rng.uniform(0,1,n)); th=rng.uniform(0,2*np.pi,n)
        x,y=center[0]+r*np.cos(th),center[1]+r*np.sin(th); ok=r>keep_out; x,y=x[ok],y[ok]
        d=d50*np.exp(rng.normal(0,0.5,len(x))); z=hz(x,y)
        for i in range(len(x)):
            V,F=(protos if d[i]>0.04 else protos0)[rng.integers(12)]
            s=d[i]/2; sx,sy,sz=s*rng.uniform(0.7,1.3),s*rng.uniform(0.7,1.3),s*rng.uniform(0.5,1.0)
            yaw=rng.uniform(0,2*np.pi); c,sn=math.cos(yaw),math.sin(yaw)
            P=V*[sx,sy,sz]; P=np.stack([c*P[:,0]-sn*P[:,1],sn*P[:,0]+c*P[:,1],P[:,2]],1)+[x[i],y[i],z[i]-0.35*sz*1.1]
            k=int(rng.integers(len(PEB_COLS))); parts[k][0].append(P); parts[k][1].append(F); count+=1
    files,tris=_emit(out_dir,'pebbles',parts,PEB_COLS)
    return files,tris,count

def model(models_dir, name, files, cast_shadows):
    d=Path(models_dir)/name
    vis=''.join(f'<visual name="v{i}"><cast_shadows>{"true" if cast_shadows else "false"}</cast_shadows><geometry><mesh><uri>model://{name}/meshes/{f}</uri></mesh></geometry></visual>' for i,f in enumerate(files))
    (d/'model.sdf').write_text(f'<?xml version="1.0"?><sdf version="1.11"><model name="{name}"><static>true</static><link name="link">{vis}</link></model></sdf>')
    (d/'model.config').write_text(f'<?xml version="1.0"?><model><name>{name}</name><sdf version="1.11">model.sdf</sdf></model>')
