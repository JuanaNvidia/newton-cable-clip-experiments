"""Sample connector surfaces against socket solids and gripper pads."""
from pathlib import Path
from motion_io import load_motion,motion_sha256
import argparse,json,hashlib
import numpy as np,trimesh
from scipy.spatial.transform import Rotation as R
from assets import plug_parts,fixture_parts,SEAT
from scene import SOCKET_POSITIONS,SOCKET_YAWS
from robot import fingertip_lip
root=Path(__file__).resolve().parent;p=argparse.ArgumentParser();p.add_argument('--motion',default='motion');a=p.parse_args();d=load_motion(root/(a.motion+'.npz'));q=d['poses'];plugs=d['plug_bodies'];names=list(d['robot_names']);robot=int(d['robot_start'])
def surface(part):
 size=np.array(part['size']);result=[]
 for axis in range(3):
  other=[j for j in range(3) if j!=axis];grids=[np.linspace(-size[j]/2,size[j]/2,max(3,int(np.ceil(size[j]/.001))+1)) for j in other];uv=np.array(np.meshgrid(*grids,indexing='ij')).reshape(2,-1).T
  for sign in [-1,1]:
   pts=np.empty((len(uv),3));pts[:,axis]=sign*size[axis]/2;pts[:,other]=uv;result.append(pts)
 return R.from_quat(part['q']).apply(np.vstack(result))+part['pos']
points=np.vstack([surface(part) for part in plug_parts() if part['collision'] and part['kind']=='box'])
def sdf_box(pts,center,size):
 delta=abs(pts-center)-np.array(size)/2;return np.linalg.norm(np.maximum(delta,0),axis=1)+np.minimum(np.max(delta,axis=1),0)
static,moving=fixture_parts();socket_boxes=[]
for seat,yaw in zip(SOCKET_POSITIONS,SOCKET_YAWS):
 rot=R.from_euler('z',yaw);receiver=seat-rot.apply(SEAT)
 for parts,offset in [(moving,receiver),(static,receiver-rot.apply([0,-.018,.024]))]:
  for part in parts:
   if part['collision'] and part['kind']=='box':socket_boxes.append((offset+rot.apply(part['pos']),rot*R.from_quat(part['q']),part['size']))
maxima=dict(socket_boxes=0.,finger_pads=0.,finger_lips=0.);where={};lip=fingertip_lip()
for frame in range(0,len(q),4):
 b=q[frame]
 for k,body in enumerate(plugs):
  pts=R.from_quat(b[body,3:]).apply(points)+b[body,:3]
  def update(key,val):
   if val>maxima[key]:maxima[key]=val;where[key]=dict(frame=frame,connector=k+1)
  for center,rot,size in socket_boxes:
   if np.linalg.norm(b[body,:3]-center)>.1:continue
   local=rot.inv().apply(pts-center);update('socket_boxes',max(0,float(-sdf_box(local,[0,0,0],size).min()*1000)))
  for side in ['left','right']:
   pad=b[robot+names.index(side+'_pad')]
   if np.linalg.norm(pad[:3]-b[body,:3])>.10:continue
   local=R.from_quat(pad[3:]).inv().apply(pts-pad[:3]);near=(abs(local[:,0])<.012)&(local[:,1]>-.010)&(local[:,1]<.002)&(local[:,2]>-.001)&(local[:,2]<.039);local=local[near]
   if not len(local):continue
   update('finger_pads',max(0,float(-sdf_box(local,[0,-.0026,.01875],[.022,.008,.0375]).min()*1000)))
   lipnear=local[(local[:,1]<-.006)&(local[:,2]>.035)]
   if False and len(lipnear):update('finger_lips',max(0,float(trimesh.proximity.signed_distance(lip,lipnear).max()*1000)))
r=dict(motion_sha256=motion_sha256(root/(a.motion+'.npz')),passed=max(maxima.values())<.2,max_sampled_connector_penetration_mm=maxima,where=where,method='Connector box surface grid at <=1 mm, sampled 15 Hz, against socket box and pad/lip signed distances; excludes decorative parts and cylindrical pedestal; not swept-volume proof')
(root/('connector_contact_validation.json' if a.motion=='motion' else a.motion+'_connector_contact_validation.json')).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
