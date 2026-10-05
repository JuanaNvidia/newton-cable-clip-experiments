"""Offscreen rendering of recorded Newton states, not a physics rerun."""
from pathlib import Path
import argparse,subprocess,json
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial.transform import Rotation
import warp as wp
import newton,newton.viewer
from assets import plug_parts,fixture_parts,SEAT
from robot import visual_meshes
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--strategy',default='flat');a=p.parse_args();d=np.load(ROOT/(a.strategy+'.npz'));poses=d['poses'];phases=d['phases'];meta=json.loads((ROOT/(a.strategy+'.json')).read_text());rv=visual_meshes(meta.get('existing_fingertip_support_lips',True));title='Side-tab grasp and angled insertion' if meta['strategy']=='direct' else 'Side-tab grasp, release, then press'
wp.init();b=newton.ModelBuilder();cfg=newton.ModelBuilder.ShapeConfig(density=0.,has_shape_collision=False,has_particle_collision=False)
def parts(items,body,offset=np.zeros(3)):
 for part in items:
  kw=dict(body=body,xform=wp.transform(wp.vec3(*(np.array(part['pos'])+offset)),wp.quat(*part['q'])),cfg=cfg,color=tuple(part['color']))
  if part['kind']=='box':b.add_shape_box(**kw,hx=part['size'][0]/2,hy=part['size'][1]/2,hz=part['size'][2]/2)
  else:b.add_shape_cylinder(**kw,radius=part['size'][0],half_height=part['size'][1]/2)
seat=np.array([-.025,.16,.0305]);receiver=seat-SEAT;static,moving=fixture_parts();parts(moving,-1,receiver);parts(static,-1,receiver-[0,-.018,.024])
b.add_shape_box(body=-1,hx=.325,hy=.35,hz=.010,xform=wp.transform(wp.vec3(.025,.30,-.009),wp.quat_identity()),cfg=cfg,color=(.66,.47,.27))
b.add_shape_box(body=-1,hx=.12,hy=.12,hz=.35,xform=wp.transform(wp.vec3(.5,.52,-.349),wp.quat_identity()),cfg=cfg,color=(.25,.27,.30))
for k,q in enumerate(poses[0]):
 body=b.add_link(xform=wp.transform(wp.vec3(*q[:3]),wp.quat(*q[3:])),is_kinematic=True)
 if k==0:parts(plug_parts(),body)
 else:
  for mesh,color in rv[k-1]:
   tri=np.asarray(mesh.vertices)[np.asarray(mesh.faces)];n=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);n/=np.maximum(np.linalg.norm(n,axis=1,keepdims=True),1e-12)
   b.add_shape_mesh(body=body,mesh=newton.Mesh(tri.reshape(-1,3),np.arange(tri.size//3,dtype=np.int32),normals=np.repeat(n,3,axis=0),compute_inertia=False),cfg=cfg,color=tuple(color))
model=b.finalize(device='cpu');state=model.state();viewer=newton.viewer.ViewerGL(width=1120,height=840,headless=True,enable_cuda_interop=newton.viewer.ViewerGL.CudaInterop(0));viewer.set_model(model);viewer.camera.near=.0001;viewer.camera.far=5;viewer.camera.fov=38;viewer.renderer.draw_sky=False
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',20);small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',17)
frames=ROOT/'work'/('frames_'+a.strategy);frames.mkdir(parents=True,exist_ok=True)
def camera(target,offset):
 eye=target+offset;v=-offset/np.linalg.norm(offset);viewer.set_camera(wp.vec3(*eye),pitch=float(np.degrees(np.arcsin(v[2]))),yaw=float(np.degrees(np.arctan2(v[1],v[0]))))
def shot(i,wide=False):
 target=np.array([.1,.27,.05]) if wide else poses[i,0,:3]+[0,0,.015];camera(target,np.array([.85,-1.,.90]) if wide else np.array([.19,-.24,.18]));viewer.begin_frame(i/60);viewer.log_state(state);viewer.end_frame();return Image.fromarray(viewer.get_frame().numpy())
try:
 for n,i in enumerate(range(0,len(poses),2)):
  state.body_q.assign(poses[i]);im=shot(i);overview=shot(i,True).resize((280,210));im.paste(overview,(840,80));draw=ImageDraw.Draw(im);draw.rectangle((0,0,1120,70),fill=(15,22,30));draw.text((16,10),f'No top rib | {title} | Newton VBD',font=font,fill='white');draw.text((16,40),f'{i/60:.2f}s: {phases[i]}',font=small,fill=(120,210,235));draw.rectangle((0,795,1120,840),fill=(15,22,30));draw.text((16,807),('Contact-only grasp | '+('Existing fingertip lips' if meta.get('existing_fingertip_support_lips',True) else 'Pads only; no fingertip lips')+' | No cables or latch constraint'),font=small,fill=(255,205,110));im.save(frames/f'frame_{n:04d}.png')
  if i in [0,510,600,1198]:im.save(ROOT/f'{a.strategy}_{i:04d}.png')
  if n%120==0:print('render',a.strategy,n,flush=True)
 subprocess.run(['ffmpeg','-y','-v','error','-framerate','30','-i',str(frames/'frame_%04d.png'),'-vf','tpad=stop_duration=2:stop_mode=clone','-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(ROOT/(a.strategy+'.mp4'))],check=True)
finally:viewer.close()
