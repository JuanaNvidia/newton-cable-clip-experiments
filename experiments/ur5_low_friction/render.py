"""Render generated assets and recorded Newton physics without running Isaac Kit."""
from pathlib import Path
from motion_io import load_motion,motion_sha256
from audit_geometry import retained_at_clips
from scene import TABLE_BOXES,TABLE_LEGS,SOCKET_POSITIONS,SOCKET_YAWS,CLIP_ORIGINS,CLIP_YAWS,CLIP_CENTERS
import argparse,subprocess,json,hashlib,fcntl
import numpy as np
from robot import visual_meshes
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw,ImageFont
import warp as wp
import newton,newton.viewer
from clip_geometry import CLIP_ORIGIN,HOLE_BOTTOM,clip_meshes
from assets import plug_parts,fixture_parts,RECEIVER_ORIGIN,OPEN_DEG
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--stills-only',action='store_true');p.add_argument('--motion',default='motion');p.add_argument('--stride',type=int,default=2);p.add_argument('--output',default='ur5_cable_clip');p.add_argument('--cache-only',action='store_true',help='Prepare verified frame cache without publishing a partial video');a=p.parse_args()
d=load_motion(ROOT/(a.motion+'.npz'));poses=d['poses'];fps=int(d['fps']);N=int(d['segments']);ds=float(d['segment_length']);TOPS=list(d['clip_top_bodies']);PLUGS=list(d['plug_bodies']);ROBOT=int(d['robot_start']);BODY_COUNT=poses.shape[1];robot_visuals=visual_meshes()
phase_start={str(name):int(np.where(d['phases']==name)[0][0])/fps for name in np.unique(d['phases'])}
metadata=json.loads((ROOT/(a.motion+'.json')).read_text()) if (ROOT/(a.motion+'.json')).exists() else {};spring_label='original spring' if metadata.get('clip_spring_stiffness_Nm_rad',.01146)<.02 else 'preloaded spring'
report_path=ROOT/('validation.json' if a.motion=='motion' else a.motion+'_validation.json');report=json.loads(report_path.read_text()) if report_path.exists() else {}
if report.get('motion_sha256') != motion_sha256(ROOT/(a.motion+'.npz')):report={}
material_stages=metadata.get('solver_runs')
if material_stages is None and (ROOT/(a.motion+'.npz.json')).exists():material_stages=json.loads((ROOT/(a.motion+'.npz.json')).read_text()).get('solver_runs')
if material_stages is None:material_stages=[dict(start_frame=0,clip_friction=.5),dict(start_frame=1890,clip_friction=.1)]
def clip_mu_at_frame(i):
 return next(stage.get('clip_friction',.5) for stage in reversed(material_stages) if i>=stage['start_frame'])
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
static,moving=fixture_parts()
from assets import SEAT
for seat,yaw in zip(SOCKET_POSITIONS,SOCKET_YAWS):
 rot=Rotation.from_euler('z',yaw);receiver=seat-rot.apply(SEAT)
 parts(moving,-1,receiver,rot);parts(static,-1,receiver-rot.apply([0,-.018,.024]),rot)
for j,(center,size) in enumerate(TABLE_BOXES):
 b.add_shape_box(body=-1,hx=size[0]/2,hy=size[1]/2,hz=size[2]/2,xform=wp.transform(wp.vec3(*center),wp.quat_identity()),cfg=cfg,color=(.66,.47,.27) if j==0 else (.47,.51,.54))
meshes=clip_meshes()
def clipmesh(data,body,offset=None,color=(.50,.54,.58),rotation=None):
 v,f=data
 if rotation is not None:v=rotation.apply(v)
 tri=v[f.reshape(-1,3)];normals=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
 v=tri.reshape(-1,3);f=np.arange(len(v),dtype=np.int32)
 b.add_shape_mesh(body=body,mesh=newton.Mesh(v,f,normals=np.repeat(normals,3,axis=0),compute_inertia=False),xform=wp.transform(wp.vec3(*(np.zeros(3) if offset is None else offset)),wp.quat_identity()),cfg=cfg,color=color)
for origin,yaw in zip(CLIP_ORIGINS,CLIP_YAWS):
 rot=Rotation.from_euler('z',yaw);clipmesh(meshes['bottom'],-1,origin,rotation=rot)
 b.add_shape_cylinder(body=-1,xform=wp.transform(wp.vec3(*(origin+rot.apply(HOLE_BOTTOM))),wp.quat(*(rot*Rotation.from_euler('x',90,degrees=True)).as_quat())),radius=.0018,half_height=.016,cfg=cfg,color=(.7,.72,.75))
rod_colors={int(body):COLORS[c] for c,chain in enumerate(d['rod_ids'].reshape(6,-1)) for body in chain}
for k,q in enumerate(poses[0]):
 body=b.add_link(xform=wp.transform(wp.vec3(*q[:3]),wp.quat(*q[3:])),is_kinematic=True);assert body==k
 if k in PLUGS:parts(plug_parts(),body)
 elif k in TOPS:clipmesh(meshes['top'],body,color=(.65,.68,.71))
 elif k>=ROBOT:
  for m,color in robot_visuals[k-ROBOT]:clipmesh((np.asarray(m.vertices),np.asarray(m.faces).flatten()),body,color=tuple(color))
 else:b.add_shape_capsule(body=body,radius=.001,half_height=ds/2,cfg=cfg,color=rod_colors[k])
# Robot base pedestal and bench legs, visual only.
b.add_shape_box(body=-1,hx=.12,hy=.12,hz=.35,xform=wp.transform(wp.vec3(.50,.52,-.349),wp.quat_identity()),cfg=cfg,color=(.22,.25,.28))
for xx,yy in TABLE_LEGS:b.add_shape_box(body=-1,hx=.012,hy=.012,hz=.35,xform=wp.transform(wp.vec3(xx,yy,-.36),wp.quat_identity()),cfg=cfg,color=(.3,.33,.36))
model=b.finalize(device='cpu');state=model.state();viewer=newton.viewer.ViewerGL(width=1280,height=960,headless=True,enable_cuda_interop=newton.viewer.ViewerGL.CudaInterop(0));viewer.set_model(model);viewer.camera.near=.0001;viewer.camera.far=5;viewer.camera.fov=38;viewer.renderer.draw_sky=False;viewer.renderer.sky_upper=(.86,.89,.92)
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',23);small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',20)
frames=ROOT/'work'/('frames_'+a.output);frames.mkdir(parents=True,exist_ok=True)
def camera(wide=False,t=0.):
 eye=np.array([1.24,-.94,1.06]);target=np.array([.06,.36,.07])
 phase=str(d['phases'][min(round(t*fps),len(poses)-1)])
 if not wide and phase.startswith('connector_'):
  k=int(phase.rsplit('_',1)[1])-1;body=poses[min(int(t*fps),len(poses)-1),PLUGS[k],:3];target=body+np.array([0,0,.015]);eye=target+np.array([.27,-.31,.25])
 elif not wide and phase.startswith(('clip_','repair_')):
  k=int(phase.rsplit('_',1)[1])-1;target=CLIP_CENTERS[k]+np.array([.025,.022,.01]);eye=target+np.array([.24,-.20,.23])
 v=(target-eye)/np.linalg.norm(target-eye);viewer.set_camera(wp.vec3(*eye),pitch=float(np.degrees(np.arcsin(v[2]))),yaw=float(np.degrees(np.arctan2(v[1],v[0]))))
def frame(i,wide=False):
 camera(wide,i/fps)
 q=poses[i]
 state.body_q.assign(q);viewer.begin_frame(i/fps);viewer.log_state(state);viewer.end_frame();im=Image.fromarray(viewer.get_frame().numpy());draw=ImageDraw.Draw(im);draw.rectangle((0,0,1280,86),fill=(15,22,30))
 draw.text((20,12),('Diagnostic clip test: connectors initialized in sockets' if a.motion.startswith('clip_trial') else 'UR5 + Robotiq pads | 6 x 2 mm cables, 762 mm | Newton'),font=font,fill='white')
 t=i/fps
 recorded=str(d['phases'][i]);phase='Harness resting on table';u=float(d['phase_elapsed'][i]) if 'phase_elapsed' in d else t-phase_start[recorded]
 if recorded.startswith('connector_'):
  k=int(recorded.rsplit('_',1)[1])-1
  phase=f'Connector {k+1}'+(' retry' if 'attempt_history' in d and d['attempt_history'][i]>1 else '')+': '+('approach' if u<2.7 else 'grasp existing side tabs' if u<3.6 else 'lift and transport' if u<7 else 'lower at 25 degrees' if u<8.6 else 'rotate into socket' if u<10.4 else 'check latch, release and withdraw')
 elif recorded.startswith('clip_'):
  k=int(recorded.rsplit('_',1)[1])-1
  phase=f'Clip {k+1}: '+('approach bundle' if u<2.7 else 'close contact grasp' if u<3.6 else 'lift and align' if u<6 else 'feed cables through mouth' if u<10 else 'release and withdraw')
 elif recorded.startswith('repair_'):
  k=int(recorded.rsplit('_',1)[1])-1;phase=f'Clip {k+1}: opposite-side missed-wire recovery; '+('approach' if u<2.7 else 'grasp' if u<3.6 else 'pull into channel' if u<10 else 'release and withdraw')
 elif recorded.startswith('wrist_recovery'):phase='Unwind wrist in free space; connectors locked'
 elif t>15:phase='Observe after insertion attempts'
 draw.text((20,48),f'{phase} | {t:.2f} s | Scripted arm; contact-only grasp',font=small,fill=(125,214,237))
 if not wide and t>=3.0:
  camera(True,t);viewer.begin_frame(i/fps);viewer.log_state(state);viewer.end_frame();overview=Image.fromarray(viewer.get_frame().numpy()).resize((320,240),Image.Resampling.LANCZOS);im.paste(overview,(940,100));draw=ImageDraw.Draw(im);draw.rectangle((939,99,1261,341),outline=(15,22,30),width=2);draw.rectangle((940,100,1260,126),fill=(15,22,30));draw.text((948,102),'Full arm view',font=small,fill='white')
 inside=np.sum(retained_at_clips(q,d['rod_ids'],ds,TOPS),axis=1);latched=d['latch_history'][i];draw.rectangle((0,912,1280,960),fill=(15,22,30));draw.text((20,925),f'Clip counts: {inside[0]}/6, {inside[1]}/6, {inside[2]}/6 | Sockets latched: {int(sum(latched))}/2 | Clip μ={clip_mu_at_frame(i):g} | Experimental',font=small,fill=(255,205,100))
 return im
try:
 if not a.stills_only:
  # Cache keys include renderer/asset bytes and every per-frame input used below.
  dependencies=[ROOT/name for name in ['render.py','robot.py','assets.py','scene.py','clip_geometry.py','audit_geometry.py','retention.py','clipTop.stl','clipBottom.stl','geometry_check.json','actual_clip_scene.usda']]
  dependencies+=sorted(p for p in (ROOT/'robot_assets').rglob('*') if p.is_file() and p.suffix in ['.stl','.dae','.urdf'])
  dependency_hash=hashlib.sha256(b''.join(p.read_bytes() for p in dependencies)+d['rod_ids'].tobytes()+str((N,ds,ROBOT,TOPS,PLUGS,getattr(newton,'__version__',''),wp.__version__)).encode()).digest()
  cache_lock=(frames/'cache.lock').open('a');fcntl.flock(cache_lock,fcntl.LOCK_EX)
  manifest_path=frames/'cache.json';cache=json.loads(manifest_path.read_text()) if manifest_path.exists() else {};valid={}
  for n,i in enumerate(range(0,len(poses),a.stride)):
   path=frames/f'frame_{n:04}.png';key=hashlib.sha256(dependency_hash+poses[i].tobytes()+d['latch_history'][i].tobytes()+str((i,fps,str(d['phases'][i]),float(d['phase_elapsed'][i]) if 'phase_elapsed' in d else i/fps-phase_start[str(d['phases'][i])],int(d['attempt_history'][i]) if 'attempt_history' in d else 1,a.motion.startswith('clip_trial'))).encode()).hexdigest()
   entry=cache.get(str(n));hit=entry and entry['input_sha256']==key and path.exists() and hashlib.sha256(path.read_bytes()).hexdigest()==entry['png_sha256']
   if not hit:frame(i).save(path)
   valid[str(n)]={'input_sha256':key,'png_sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
   if n%60==0:manifest_path.write_text(json.dumps(valid));print('FRAME CACHE',n,'reused' if hit else 'rendered',flush=True)
  manifest_path.write_text(json.dumps(valid))
  if a.cache_only:raise SystemExit(0)
  for old in frames.glob('frame_*.png'):
   if int(old.stem.split('_')[-1])>=len(valid):old.unlink()
  subprocess.run(['ffmpeg','-y','-v','error','-framerate',str(fps/a.stride),'-i',str(frames/'frame_%04d.png'),'-vf','tpad=start_duration=1.5:stop_duration=2:start_mode=clone:stop_mode=clone','-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(ROOT/(a.output+'.mp4'))],check=True)
 prefix='' if a.output=='ur5_cable_clip' else a.output+'_'
 stills=[('before',1.4),('seated',(len(poses)-1)/fps)]
 for k,name in enumerate(['first_connector','second_connector']):
  indices=np.where(d['latch_history'][:,k])[0]
  if len(indices):stills.append((name,min((indices[0]+120)/fps,(len(poses)-1)/fps)))
 for k in range(3):
  key=f'clip_{k+1}'
  if key in phase_start:stills.append((f'clip{k+1}',min(phase_start[key]+12.2,(len(poses)-1)/fps)))
 for name,t in stills:
  if t*fps<len(poses):frame(round(t*fps)).save(ROOT/(prefix+name+'.png'))
 frame(min(84,len(poses)-1),True).save(ROOT/(prefix+'assembly_before.png'));frame(len(poses)-1,True).save(ROOT/(prefix+'assembly_after.png'))
finally:viewer.close()
