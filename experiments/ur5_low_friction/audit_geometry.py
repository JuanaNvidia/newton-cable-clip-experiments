import numpy as np
from scipy.spatial.transform import Rotation
from scene import CLIP_ORIGINS,CLIP_YAWS
from retention import lid_ceiling
def retained_at_clips(q,rod_ids,DS,tops):
 ids=np.array(rod_ids).reshape(6,-1);N=ids.shape[1]
 result=[]
 for origin,yaw,top in zip(CLIP_ORIGINS,CLIP_YAWS,tops):
  rot=Rotation.from_euler('z',yaw);mask=[]
  for chain in ids:
   qq=q[chain];v=Rotation.from_quat(qq[:,3:]).apply(np.tile([0,0,DS/2],(N,1)))
   aa=rot.inv().apply(qq[:,:3]-v-origin);bb=rot.inv().apply(qq[:,:3]+v-origin);options=[]
   for a,z in zip(aa,bb):
    if (a[1]+.015)*(z[1]+.015)<=0 and abs(z[1]-a[1])>1e-10:options.append(a+(z-a)*(-.015-a[1])/(z[1]-a[1]))
   if not options:mask.append(False);continue
   pt=min(options,key=lambda x:abs(x[0]-.018));world=origin+rot.apply(pt);lp=Rotation.from_quat(q[top,3:]).inv().apply(world-q[top,:3]);roof=lid_ceiling(np.array([lp[0]]))[0]
   mask.append(bool(.005<pt[0]<.028 and pt[2]>.0118 and np.isfinite(roof) and roof-lp[2]>.0008))
  result.append(mask)
 return result
