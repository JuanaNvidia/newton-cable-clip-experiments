"""Original-mesh center-plane retention geometry, shared by control and audit."""
from pathlib import Path
import numpy as np,trimesh
from scipy.spatial.transform import Rotation as R
from clip_geometry import CLIP_ORIGIN
ROOT=Path(__file__).resolve().parent
mesh=trimesh.load(ROOT/'clipTop.stl');mesh.apply_scale(.001)
section=mesh.section(plane_origin=[0,-.015,0],plane_normal=[0,1,0])
EDGES=np.concatenate([np.stack([path[:-1],path[1:]],axis=1) for path in section.discrete])
def lid_ceiling(xs):
 roof=np.full(len(xs),np.inf)
 for a,b in EDGES:
  if abs(b[0]-a[0])<1e-12:continue
  mask=(xs>=min(a[0],b[0]))&(xs<=max(a[0],b[0]));z=a[2]+(xs-a[0])*(b[2]-a[2])/(b[0]-a[0]);roof=np.where(mask,np.minimum(roof,z),roof)
 return roof

def crossings(q,rod_ids,ds,plane):
 ids=np.asarray(rod_ids).reshape(6,-1);points=[];indices=[];directions=[]
 for chain in ids:
  rods=q[chain];v=R.from_quat(rods[:,3:]).apply(np.tile([0,0,ds/2],(len(chain),1)));starts=rods[:,:3]-v;ends=rods[:,:3]+v
  options=[]
  for k,(a,b) in enumerate(zip(starts,ends)):
   if (a[1]-plane)*(b[1]-plane)<=0 and abs(b[1]-a[1])>1e-9:
    point=a+(b-a)*(plane-a[1])/(b[1]-a[1]);options.append((point,k))
  if options:
   point,k=min(options,key=lambda row:abs(row[0][0]));points.append(point);indices.append(k);directions.append(v[k]*2/ds)
  else:points.append([np.nan]*3);indices.append(-1);directions.append([np.nan]*3)
 return np.array(points),np.array(indices),np.array(directions)

def retained_at_clip(q,rod_ids,ds,top):
 points,_,_=crossings(q,rod_ids,ds,CLIP_ORIGIN[1]-.015)
 local=points-CLIP_ORIGIN;lidlocal=R.from_quat(q[top,3:]).inv().apply(points-q[top,:3]);roof=lid_ceiling(lidlocal[:,0]);clearance=np.where(np.isfinite(roof),roof-lidlocal[:,2],-np.inf)
 inside=(local[:,0]>.005)&(local[:,0]<.028)&(local[:,2]>.0118)&(clearance>.0008)
 return inside,points,clearance
