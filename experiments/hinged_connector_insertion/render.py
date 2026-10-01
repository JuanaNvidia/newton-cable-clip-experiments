"""Render generated assets and recorded Newton physics without running Isaac Kit."""
from pathlib import Path
import argparse,subprocess
import numpy as np
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw,ImageFont
import warp as wp
import newton,newton.viewer
from assets import plug_parts,fixture_parts,PIVOT,OPEN_DEG
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--stills-only',action='store_true');p.add_argument('--motion',default='motion');a=p.parse_args()
d=np.load(ROOT/(a.motion+'.npz'));poses=d['poses'];fps=int(d['fps']);N=int(d['segments']);ds=float(d['segment_length']);far=d['far_position']
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
b.add_shape_box(body=-1,hx=.19,hy=.24,hz=.008,xform=wp.transform(wp.vec3(0,.12,-.010),wp.quat_identity()),cfg=cfg,color=(.66,.47,.27))
for center,extent in [((-.113,.12,-.0005),(.154,.48,.003)),((.113,.12,-.0005),(.154,.48,.003)),((0,-.078,-.0005),(.072,.084,.003)),((0,.198,-.0005),(.072,.324,.003))]:b.add_shape_box(body=-1,hx=extent[0]/2,hy=extent[1]/2,hz=extent[2]/2,xform=wp.transform(wp.vec3(*center),wp.quat_identity()),cfg=cfg,color=(.47,.51,.54))
for k,q in enumerate(poses[0]):
 body=b.add_link(xform=wp.transform(wp.vec3(*q[:3]),wp.quat(*q[3:])),is_kinematic=True);assert body==k
 if k==0:parts(moving,body)
 elif k==1:parts(plug_parts(),body)
 else:b.add_shape_capsule(body=body,radius=.001,half_height=ds/2,cfg=cfg,color=COLORS[(k-2)//N])
model=b.finalize(device='cpu');state=model.state();viewer=newton.viewer.ViewerGL(width=1280,height=960,headless=True,enable_cuda_interop=newton.viewer.ViewerGL.CudaInterop(0));viewer.set_model(model);viewer.camera.near=.0001;viewer.camera.far=5;viewer.camera.fov=38;viewer.renderer.draw_sky=False;viewer.renderer.sky_upper=(.86,.89,.92)
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',23);small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',20)
frames=ROOT/'work/frames';frames.mkdir(parents=True,exist_ok=True)
def camera(wide=False,t=0.):
 u=0. if wide else float(np.clip((t-.3)/1.5,0,1));u=u*u*(3-2*u)
 eye=(1-u)*np.array([.31,-.27,.25])+u*np.array([.14,-.165,.14]);target=(1-u)*np.array([0,.10,.060])+u*np.array([0,.012,.051]);v=(target-eye)/np.linalg.norm(target-eye);viewer.set_camera(wp.vec3(*eye),pitch=float(np.degrees(np.arcsin(v[2]))),yaw=float(np.degrees(np.arctan2(v[1],v[0]))))
def frame(i,wide=False):
 camera(wide,i/fps);state.body_q.assign(poses[i]);viewer.begin_frame(i/fps);viewer.log_state(state);viewer.end_frame();im=Image.fromarray(viewer.get_frame().numpy());draw=ImageDraw.Draw(im);draw.rectangle((0,0,1280,86),fill=(15,22,30))
 draw.text((20,12),'Newton | Surrogate cable plug + hinged table connector',font=font,fill='white')
 t=i/fps;phase='Align and lower plug' if t<2.7 else 'Press and rock into seat' if t<4.8 else 'Hand releases' if t<5.2 else 'Passive settling after release'
 draw.text((20,48),f'{phase} | {t:.2f} s | Approximate geometry from reference video',font=small,fill=(125,214,237));return im
try:
 if not a.stills_only:
  for n,i in enumerate(range(0,len(poses),2)):
   frame(i).save(frames/f'frame_{n:04}.png')
   if n%60==0:print('RENDER',n,flush=True)
  subprocess.run(['ffmpeg','-y','-v','error','-framerate','30','-i',str(frames/'frame_%04d.png'),'-frames:v',str(len(range(0,len(poses),2))),'-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(ROOT/'connector_insertion.mp4')],check=True)
 for name,i in [('before',0),('engaging',min(150,len(poses)-1)),('seated',len(poses)-1)]:frame(i).save(ROOT/(name+'.png'))
 frame(0,True).save(ROOT/'assembly_before.png');frame(len(poses)-1,True).save(ROOT/'assembly_after.png')
finally:viewer.close()
