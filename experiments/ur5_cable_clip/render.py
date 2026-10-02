"""Render generated assets and recorded Newton physics without running Isaac Kit."""
from pathlib import Path
import argparse,subprocess,json,hashlib
import numpy as np
from robot import visual_meshes
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw,ImageFont
import warp as wp
import newton,newton.viewer
from clip_geometry import CLIP_ORIGIN,HOLE_BOTTOM,clip_meshes
from assets import plug_parts,fixture_parts,RECEIVER_ORIGIN,OPEN_DEG
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--stills-only',action='store_true');p.add_argument('--motion',default='motion');p.add_argument('--output',default='ur5_cable_clip');a=p.parse_args()
d=np.load(ROOT/(a.motion+'.npz'));poses=d['poses'];fps=int(d['fps']);N=int(d['segments']);ds=float(d['segment_length']);TOP=int(d['clip_top_body']);ROBOT=int(d['robot_start']);BODY_COUNT=poses.shape[1];robot_visuals=visual_meshes()
metadata=json.loads((ROOT/(a.motion+'.json')).read_text()) if (ROOT/(a.motion+'.json')).exists() else {};spring_label='original spring' if metadata.get('clip_spring_stiffness_Nm_rad',.01146)<.02 else 'preloaded spring'
report_path=ROOT/('validation.json' if a.motion=='motion' else a.motion+'_validation.json');report=json.loads(report_path.read_text()) if report_path.exists() else {}
if report.get('motion_sha256') != hashlib.sha256((ROOT/(a.motion+'.npz')).read_bytes()).hexdigest():report={}
release_start=metadata.get('release_start_seconds',9.0);release_end=metadata.get('release_complete_seconds',9.6)
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
static,moving=fixture_parts();parts(static,-1)
b.add_shape_box(body=-1,hx=.19,hy=.1675,hz=.008,xform=wp.transform(wp.vec3(-.12,.0475,-.010),wp.quat_identity()),cfg=cfg,color=(.66,.47,.27))
for center,extent in [((-.173,.0475,-.0005),(.274,.335,.003)),((.053,.0475,-.0005),(.034,.335,.003)),((0,-.078,-.0005),(.072,.084,.003)),((0,.1255,-.0005),(.072,.179,.003))]:b.add_shape_box(body=-1,hx=extent[0]/2,hy=extent[1]/2,hz=extent[2]/2,xform=wp.transform(wp.vec3(*center),wp.quat_identity()),cfg=cfg,color=(.47,.51,.54))
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
 elif k>=ROBOT:
  for m,color in robot_visuals[k-ROBOT]:clipmesh((np.asarray(m.vertices),np.asarray(m.faces).flatten()),body,color=tuple(color))
 else:b.add_shape_capsule(body=body,radius=.001,half_height=ds/2,cfg=cfg,color=COLORS[(k-2)//N])
# Robot base pedestal and bench legs, visual only.
b.add_shape_box(body=-1,hx=.12,hy=.12,hz=.35,xform=wp.transform(wp.vec3(.50,.52,-.349),wp.quat_identity()),cfg=cfg,color=(.22,.25,.28))
for xx in [-.28,.04]:
 for yy in [-.07,.18]:b.add_shape_box(body=-1,hx=.012,hy=.012,hz=.35,xform=wp.transform(wp.vec3(xx,yy,-.36),wp.quat_identity()),cfg=cfg,color=(.3,.33,.36))
model=b.finalize(device='cpu');state=model.state();viewer=newton.viewer.ViewerGL(width=1280,height=960,headless=True,enable_cuda_interop=newton.viewer.ViewerGL.CudaInterop(0));viewer.set_model(model);viewer.camera.near=.0001;viewer.camera.far=5;viewer.camera.fov=38;viewer.renderer.draw_sky=False;viewer.renderer.sky_upper=(.86,.89,.92)
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',23);small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',20)
frames=ROOT/'work'/('frames_'+a.output);frames.mkdir(parents=True,exist_ok=True)
def camera(wide=False,t=0.):
 keys=[(0,[1.30,-1.20,.90],[.24,.25,.18]),(1.5,[1.30,-1.20,.90],[.24,.25,.18]),(3.0,[.14,.38,.12],[.03,.235,-.005]),(5.5,[.14,.38,.12],[.03,.235,-.005]),(6.5,[.10,.37,.13],[0,.213,.02]),(release_end+.2,[.10,.37,.13],[0,.213,.02]),(release_end+1.8,[.10,.37,.13],[0,.213,.02])]
 eye=np.array(keys[0][1]);target=np.array(keys[0][2])
 if not wide:
  for a,bk in zip(keys[:-1],keys[1:]):
   u=float(np.clip((t-a[0])/(bk[0]-a[0]),0,1));u=u*u*(3-2*u)
   eye=(1-u)*np.array(a[1])+u*np.array(bk[1]);target=(1-u)*np.array(a[2])+u*np.array(bk[2])
   if t<=bk[0]:break
 v=(target-eye)/np.linalg.norm(target-eye);viewer.set_camera(wp.vec3(*eye),pitch=float(np.degrees(np.arcsin(v[2]))),yaw=float(np.degrees(np.arctan2(v[1],v[0]))))
def frame(i,wide=False):
 camera(wide,i/fps)
 q=poses[i]
 state.body_q.assign(q);viewer.begin_frame(i/fps);viewer.log_state(state);viewer.end_frame();im=Image.fromarray(viewer.get_frame().numpy());draw=ImageDraw.Draw(im);draw.rectangle((0,0,1280,86),fill=(15,22,30))
 draw.text((20,12),'UR5 + Robotiq 2F-85 | Newton cable and contact physics',font=font,fill='white')
 t=i/fps;phase='Seated connector; free cable tails' if t<1.5 else 'Approach the cable bundle' if t<3.8 else 'Close fingers: contact grasp' if t<4.5 else 'Lift and align with clip' if t<6.3 else 'Straighten bundle outside clip' if metadata.get('straighten_before_insertion') and t<7.5 else 'Carry cables through spring gate' if t<release_start else 'Open gripper' if t<release_end else 'Open gripper stationary; cables settle'
 if 'pickup_roll_degrees' in metadata and 4.5<=t<6.3:phase='Lift and roll the held bundle level'
 if metadata.get('entry_feedback') and 10.0<=t<12.5:phase='Advance until bundle crosses into channel'
 if metadata.get('retreat_clear_seconds') and t>=release_end:
  phase='Release; let the cables settle' if t<15.5 else 'Lower open jaws clear of hanging tails' if t<16.7 else 'Withdraw gripper sideways' if t<metadata['retreat_clear_seconds'] else 'Return arm home' if t<metadata.get('home_seconds',19.5) else 'Observe cables after withdrawal'
 draw.text((20,48),f'{phase} | {t:.2f} s | Scripted arm; contact-only grasp',font=small,fill=(125,214,237))
 if not wide and t>=3.0:
  camera(True,t);viewer.begin_frame(i/fps);viewer.log_state(state);viewer.end_frame();overview=Image.fromarray(viewer.get_frame().numpy()).resize((320,240),Image.Resampling.LANCZOS);im.paste(overview,(940,100));draw=ImageDraw.Draw(im);draw.rectangle((939,99,1261,341),outline=(15,22,30),width=2);draw.rectangle((940,100,1260,126),fill=(15,22,30));draw.text((948,102),'Full arm view',font=small,fill='white')
 if t>=(len(poses)-1)/fps-1 and report:
  retained=sum(report.get('measurements',{}).get('retained_per_cable_last_second',[]));status=f'{retained}/6 cables inside clip during final second';status+=' | Numerical checks passed' if report.get('passed') else ' | Experimental: validation not passed';draw.rectangle((0,912,1280,960),fill=(15,22,30));draw.text((20,925),status,font=small,fill=(255,205,100))
 return im
try:
 if not a.stills_only:
  for old in frames.glob('frame_*.png'):old.unlink()
  for n,i in enumerate(range(min(90,len(poses)-1),len(poses),2)):
   frame(i).save(frames/f'frame_{n:04}.png')
   if n%60==0:print('RENDER',n,flush=True)
  subprocess.run(['ffmpeg','-y','-v','error','-framerate','30','-i',str(frames/'frame_%04d.png'),'-vf','tpad=start_duration=1.5:stop_duration=2:start_mode=clone:stop_mode=clone','-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(ROOT/(a.output+'.mp4'))],check=True)
 prefix='' if a.output=='ur5_cable_clip' else a.output+'_'
 for name,i in [('before',min(90,len(poses)-1)),('grasp',min(285,len(poses)-1)),('clip_open',int(np.argmin(Rotation.from_quat(poses[:,TOP,3:]).as_euler('xyz',degrees=True)[:,1]))),('seated',len(poses)-1)]:frame(i).save(ROOT/(prefix+name+'.png'))
 frame(min(90,len(poses)-1),True).save(ROOT/(prefix+'assembly_before.png'));frame(len(poses)-1,True).save(ROOT/(prefix+'assembly_after.png'))
finally:viewer.close()
