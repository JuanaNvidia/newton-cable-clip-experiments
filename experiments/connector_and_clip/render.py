"""Render generated assets and recorded Newton physics without running Isaac Kit."""
from pathlib import Path
import argparse,subprocess,json
import numpy as np
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw,ImageFont
import warp as wp
import newton,newton.viewer
from clip_geometry import CLIP_ORIGIN,HOLE_BOTTOM,clip_meshes
from assets import plug_parts,fixture_parts,RECEIVER_ORIGIN,OPEN_DEG
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--stills-only',action='store_true');p.add_argument('--motion',default='motion');p.add_argument('--output',default='connector_and_clip');a=p.parse_args()
d=np.load(ROOT/(a.motion+'.npz'));poses=d['poses'];fps=int(d['fps']);N=int(d['segments']);ds=float(d['segment_length']);far=d['far_position'];TOP=int(d['clip_top_body']);GRIPS=d['grip_indices'];BODY_COUNT=poses.shape[1]
metadata=json.loads((ROOT/(a.motion+'.json')).read_text()) if (ROOT/(a.motion+'.json')).exists() else {};spring_label='original spring' if metadata.get('clip_hinge_stiffness_Nm_rad',.01146)<.02 else 'preloaded spring'
wp.init();wp.set_device('cpu');b=newton.ModelBuilder(gravity=(0.,0.,0.));cfg=newton.ModelBuilder.ShapeConfig(density=0.,has_shape_collision=False,has_particle_collision=False)
COLORS=[(.85,.04,.065),(.95,.06,.08),(.78,.035,.06),(.93,.10,.15),(.035,.19,.8),(.13,.24,.82)]
def parts(items,body,offset=None,rotation=None):
 for part in items:
  pos=np.array(part['pos']);q=Rotation.from_quat(part['q'])
  if rotation is not None:pos=rotation.apply(pos);q=rotation*q
  if offset is not None:pos+=offset
  kw=dict(body=body,xform=wp.transform(wp.vec3(*pos),wp.quat(*q.as_quat())),cfg=cfg,color=tuple(part['color']))
  if part['kind']=='box':b.add_shape_box(**kw,hx=part['size'][0]/2,hy=part['size'][1]/2,hz=part['size'][2]/2)
  else:b.add_shape_cylinder(**kw,radius=part['size'][0],half_height=part['size'][1]/2)
static,moving=fixture_parts();parts(static,-1);parts(plug_parts(),-1,far,Rotation.from_euler('z',180,degrees=True))
b.add_shape_box(body=-1,hx=.19,hy=.34,hz=.008,xform=wp.transform(wp.vec3(0,.22,-.010),wp.quat_identity()),cfg=cfg,color=(.66,.47,.27))
for center,extent in [((-.113,.22,-.0005),(.154,.68,.003)),((.113,.22,-.0005),(.154,.68,.003)),((0,-.078,-.0005),(.072,.084,.003)),((0,.298,-.0005),(.072,.524,.003))]:b.add_shape_box(body=-1,hx=extent[0]/2,hy=extent[1]/2,hz=extent[2]/2,xform=wp.transform(wp.vec3(*center),wp.quat_identity()),cfg=cfg,color=(.47,.51,.54))
meshes=clip_meshes()
def clipmesh(data,body,offset=None,color=(.50,.54,.58)):
 v,f=data;tri=v[f.reshape(-1,3)];normals=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
 v=tri.reshape(-1,3);f=np.arange(len(v),dtype=np.int32)
 b.add_shape_mesh(body=body,mesh=newton.Mesh(v,f,normals=np.repeat(normals,3,axis=0),compute_inertia=False),xform=wp.transform(wp.vec3(*(np.zeros(3) if offset is None else offset)),wp.quat_identity()),cfg=cfg,color=color)
clipmesh(meshes['bottom'],-1,CLIP_ORIGIN)
b.add_shape_cylinder(body=-1,xform=wp.transform(wp.vec3(*(CLIP_ORIGIN+HOLE_BOTTOM)),wp.quat(*Rotation.from_euler('x',90,degrees=True).as_quat())),radius=.0018,half_height=.016,cfg=cfg,color=(.7,.72,.75))
for k,q in enumerate(poses[0]):
 body=b.add_link(xform=wp.transform(wp.vec3(*q[:3]),wp.quat(*q[3:])),is_kinematic=True);assert body==k
 if k==0:parts(moving,body)
 elif k==1:parts(plug_parts(),body)
 elif k==TOP:clipmesh(meshes['top'],body,color=(.65,.68,.71))
 else:b.add_shape_capsule(body=body,radius=.001,half_height=ds/2,cfg=cfg,color=COLORS[(k-2)//N])
for side in range(2):
 for jaw in range(2):
  body=b.add_link(is_kinematic=True,label=f'VirtualCableGrip_{side}_{jaw}')
  b.add_shape_box(body=body,hx=.0095,hy=.015,hz=.00065,cfg=cfg,color=(.04,.65,.8))
model=b.finalize(device='cpu');state=model.state();viewer=newton.viewer.ViewerGL(width=1280,height=960,headless=True,enable_cuda_interop=newton.viewer.ViewerGL.CudaInterop(0));viewer.set_model(model);viewer.camera.near=.0001;viewer.camera.far=5;viewer.camera.fov=38;viewer.renderer.draw_sky=False;viewer.renderer.sky_upper=(.86,.89,.92)
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',23);small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',20)
frames=ROOT/'work/frames';frames.mkdir(parents=True,exist_ok=True)
def camera(wide=False,t=0.):
 keys=[(0,[.43,-.34,.34],[0,.18,.105]),(1.2,[.14,-.165,.14],[0,.012,.051]),(4.8,[.14,-.165,.14],[0,.012,.051]),(6.2,[.28,-.10,.30],[0,.20,.07]),(8.0,[.10,.035,.135],[0,.20,.023]),(11.4,[.10,.035,.135],[0,.20,.023]),(13,[.43,-.34,.34],[0,.18,.105])]
 eye=np.array(keys[0][1]);target=np.array(keys[0][2])
 if not wide:
  for a,bk in zip(keys[:-1],keys[1:]):
   u=float(np.clip((t-a[0])/(bk[0]-a[0]),0,1));u=u*u*(3-2*u)
   eye=(1-u)*np.array(a[1])+u*np.array(bk[1]);target=(1-u)*np.array(a[2])+u*np.array(bk[2])
   if t<=bk[0]:break
 v=(target-eye)/np.linalg.norm(target-eye);viewer.set_camera(wp.vec3(*eye),pitch=float(np.degrees(np.arcsin(v[2]))),yaw=float(np.degrees(np.arctan2(v[1],v[0]))))
def frame(i,wide=False):
 camera(wide,i/fps)
 q=np.zeros((BODY_COUNT+4,7),np.float32);q[:BODY_COUNT]=poses[i];q[BODY_COUNT:,6]=1
 for side,seg in enumerate(GRIPS):
  center=np.mean([poses[i,2+c*N+seg,:3] for c in range(6)],axis=0)
  direction=Rotation.from_quat(poses[i,2+seg,3:]).apply([0,0,1]);xx=np.cross(direction,[0,0,1]);xx/=max(np.linalg.norm(xx),1e-8);normal=np.cross(xx,direction);rot=Rotation.from_matrix(np.column_stack([xx,direction,normal])).as_quat()
  for jaw in range(2):
   pos=center.copy()+(-1 if jaw==0 else 1)*.0023*normal
   q[BODY_COUNT+side*2+jaw,3:]=rot
   if not 0<=i/fps<11.2:pos[2]=-1.
   q[BODY_COUNT+side*2+jaw,:3]=pos
 state.body_q.assign(q);viewer.begin_frame(i/fps);viewer.log_state(state);viewer.end_frame();im=Image.fromarray(viewer.get_frame().numpy());draw=ImageDraw.Draw(im);draw.rectangle((0,0,1280,86),fill=(15,22,30))
 draw.text((20,12),f'Newton | Six 2 mm x 18 inch cables | Clip: {spring_label}',font=font,fill='white')
 t=i/fps;phase='Insert angled connector' if t<2.7 else 'Rotate plug down; receiver fixed' if t<5.5 else 'Lower cable grips beside clip' if t<7.7 else 'Pull cables through spring gate' if t<10.8 else 'Release cable grips' if t<11.2 else 'Release connector support' if t<12.2 else 'All hands released: passive settling'
 draw.text((20,48),f'{phase} | {t:.2f} s | Original clip; connector uses approximate retention',font=small,fill=(125,214,237));return im
try:
 if not a.stills_only:
  for n,i in enumerate(range(0,len(poses),2)):
   frame(i).save(frames/f'frame_{n:04}.png')
   if n%60==0:print('RENDER',n,flush=True)
  subprocess.run(['ffmpeg','-y','-v','error','-framerate','30','-i',str(frames/'frame_%04d.png'),'-frames:v',str(len(range(0,len(poses),2))),'-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(ROOT/(a.output+'.mp4'))],check=True)
 prefix='' if a.output=='connector_and_clip' else a.output+'_'
 for name,i in [('before',0),('connector_seated',min(440,len(poses)-1)),('clip_open',int(np.argmin(Rotation.from_quat(poses[:,TOP,3:]).as_euler('xyz',degrees=True)[:,1]))),('seated',len(poses)-1)]:frame(i).save(ROOT/(prefix+name+'.png'))
 frame(0,True).save(ROOT/(prefix+'assembly_before.png'));frame(len(poses)-1,True).save(ROOT/(prefix+'assembly_after.png'))
finally:viewer.close()
