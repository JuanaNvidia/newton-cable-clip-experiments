"""Two original connector grasps, three spring clips, and conditional opposite-side wire pulls."""
from pathlib import Path
import argparse,json,math,time,os
import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation,Slerp
import warp as wp
import newton
from clip_geometry import HOLE_BOTTOM,HOLE_TOP,clip_meshes
from scene import *
from robot import ARM_NAMES,GRIP_NAMES,poses as robot_poses,packed,ik,goal_flange,theta_for_gap,TOOL_ROT
from assets import SEAT,plug_parts,fixture_parts
from audit_geometry import retained_at_clips
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--output',default='motion');p.add_argument('--iterations',type=int,default=60);p.add_argument('--substeps',type=int,default=32);p.add_argument('--max-repairs-per-clip',type=int,default=6);p.add_argument('--resume');p.add_argument('--stop-after',type=int,help='Stop after this many completed phases, preserving a restart state');args=p.parse_args()
wp.init();wp.set_device('cuda:0')
R=RADIUS;EI=.0011459155902616466
b=newton.ModelBuilder(gravity=(0.,0.,-9.81))
contact=newton.ModelBuilder.ShapeConfig(density=0.,ke=1e6,kd=100.,mu=.5,gap=.0002)
visual=newton.ModelBuilder.ShapeConfig(density=0.,has_shape_collision=False,has_particle_collision=False)
def transform(p,q=None):return wp.transform(wp.vec3(*p),wp.quat_identity() if q is None else wp.quat(*q.as_quat()))
def add_parts(parts,body,offset=None,rotation=None):
 shapes=[]
 for part in parts:
  pos=np.array(part['pos']);q=Rotation.from_quat(part['q'])
  if rotation is not None:pos=rotation.apply(pos);q=rotation*q
  if offset is not None:pos+=offset
  common=dict(body=body,xform=transform(pos,q),cfg=contact if part['collision'] else visual,color=tuple(part['color']),label=part['name'])
  if part['kind']=='box':s=b.add_shape_box(**common,hx=part['size'][0]/2,hy=part['size'][1]/2,hz=part['size'][2]/2)
  else:s=b.add_shape_cylinder(**common,radius=part['size'][0],half_height=part['size'][1]/2)
  if part['collision']:shapes.append(s)
 return shapes
for center,size in TABLE_BOXES:b.add_shape_box(body=-1,hx=size[0]/2,hy=size[1]/2,hz=size[2]/2,xform=transform(center),cfg=contact)
static,moving=fixture_parts()
for seat,yaw in zip(SOCKET_POSITIONS,SOCKET_YAWS):
 rot=Rotation.from_euler('z',yaw);receiver=seat-rot.apply(SEAT)
 add_parts(moving,-1,receiver,rot)
 # The reference base was authored around a receiver at (0,-18,24) mm.
 add_parts(static,-1,receiver-rot.apply([0,-.018,.024]),rot)
# Almost flat initial harness with outlet-height transitions near each end.
u=np.linspace(0,1,12001)
def curve(span):return np.column_stack([.13+.014*np.sin(2*np.pi*u),-.001+span*u,.0023+.0147*(np.exp(-u/.035)+np.exp(-(1-u)/.035))])
def length(span):return np.linalg.norm(np.diff(curve(span),axis=0),axis=1).sum()
span=brentq(lambda span:length(span)-L,.70,.77);guide=curve(span);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(guide,axis=0),axis=1))]
points=np.column_stack([np.interp(np.linspace(0,L,N+1),arc,guide[:,k]) for k in range(3)])
plugs=[];latches=[];plug_anchors=[];plug_rots=[Rotation.identity(),Rotation.from_euler('z',np.pi)]
for k,rot in enumerate(plug_rots):
 pos=points[0 if k==0 else -1]-rot.apply([0,.009,.012])
 body=b.add_link(xform=transform(pos,rot),mass=.015,inertia=wp.mat33(*np.diag([1.2e-6,2.2e-6,3.1e-6]).reshape(-1)),label=f'EndConnector_{k+1}');plugs.append(body);add_parts(plug_parts(),body)
 seatrot=Rotation.from_euler('z',SOCKET_YAWS[k])
 j=b.add_joint_fixed(parent=-1,child=body,parent_xform=transform(SOCKET_POSITIONS[k],seatrot),child_xform=transform([0,0,0]),collision_filter_parent=False,label=f'SeatLatch_{k+1}');b.add_articulation([j]);latches.append(j)
cfg=newton.ModelBuilder.ShapeConfig(density=1000*DS/(DS+4*R/3),ke=1e6,kd=100.,mu=.5,gap=.0002)
rod_ids=[];anchor_q=[]
for c in range(COUNT):
 dx=(c-2.5)*PITCH;chain=[];joints=[];rots=[]
 for i in range(N):
  v=points[i+1]-points[i];q=Rotation.align_vectors([v],[[0,0,1]])[0];pos=(points[i]+points[i+1])/2+[dx,0,0]
  body=b.add_link(xform=transform(pos,q),label=f'Cable_{c}_{i:03}');chain.append(body);rots.append(q);b.add_shape_capsule(body=body,radius=R,half_height=DS/2,cfg=cfg)
  if i:joints.append(b.add_joint_rod(parent=chain[-2],child=body,parent_xform=transform([0,0,DS/2]),child_xform=transform([0,0,-DS/2]),stretch_stiffness=1e6,stretch_damping=.1,bend_stiffness=EI/DS,bend_damping=.005729577951308233*.005/DS))
 b.add_articulation(joints)
 for k,rot in enumerate(plug_rots):
  anchor=np.array([dx if k==0 else -dx,.009,.012]);localq=rot.inv()*rots[0 if k==0 else -1]
  j=b.add_joint_fixed(parent=plugs[k],child=chain[0 if k==0 else -1],parent_xform=transform(anchor,localq),child_xform=transform([0,0,-DS/2 if k==0 else DS/2]));b.add_articulation([j]);plug_anchors.append(anchor);anchor_q.append(localq.as_quat())
 rod_ids+=chain
meshset=clip_meshes();tops=[]
def clip_shape(data,body,cfg,offset=None,rot=None,convex=False):
 v,f=data;m=newton.Mesh(v,f)
 if convex:m=m.compute_convex_hull()
 return b.add_shape_mesh(body=body,mesh=m,cfg=cfg,xform=transform([0,0,0] if offset is None else offset,rot))
for k,(origin,yaw) in enumerate(zip(CLIP_ORIGINS,CLIP_YAWS)):
 rot=Rotation.from_euler('z',yaw)
 clip_shape(meshset['bottom'],-1,contact,origin,rot);clip_shape(meshset['support'],-1,contact,origin,rot,True)
 top=b.add_link(xform=transform(origin+rot.apply(HOLE_BOTTOM-HOLE_TOP),rot),label=f'SpringClip_{k+1}');tops.append(top)
 cc=newton.ModelBuilder.ShapeConfig(density=1000.,ke=1e6,kd=100.,mu=.5,gap=.0002,margin=0.)
 for m in meshset['convex_top']:clip_shape(m,top,cc,convex=True)
 hinge=b.add_joint_revolute(parent=-1,child=top,parent_xform=transform(origin+rot.apply(HOLE_BOTTOM),rot),child_xform=transform(HOLE_TOP),axis=wp.vec3(0,1,0),target_pos=0.,target_ke=.114591559,target_kd=.002864789,limit_lower=math.radians(-75),limit_upper=0.,collision_filter_parent=False);b.add_articulation([hinge])
# Robot bodies are driven by URDF FK. Finger poses obey the four-bar loop.
# All cable forces are gravity, rod elasticity and Newton collision contacts.
import trimesh
robot_start=len(b.body_q);robot_names=ARM_NAMES+GRIP_NAMES
robot_ids=[];robot_shapes=[]
PICKUP_YAW=0.
open_theta=theta_for_gap(.030);hold_theta=theta_for_gap(.00599)
seed=np.array([-2.42750698,-1.55334897,2.38709924,-.83375027,-3.9983033,1.57079633])
home=np.array([.15,.38,.26]);qj=ik(goal_flange(home,open_theta,Rotation.from_euler('z',PICKUP_YAW)*TOOL_ROT),seed)
initial_robot=packed(robot_poses(qj,open_theta))
for name,q in zip(robot_names,initial_robot):
 body=b.add_link(xform=wp.transform(wp.vec3(*q[:3]),wp.quat(*q[3:])),is_kinematic=True,mass=1.,inertia=wp.mat33(*np.eye(3).reshape(-1)),label='UR5_'+name);robot_ids.append(body)
 if name in ARM_NAMES:
  from robot import URDF,origin
  link=next(x for x in URDF.findall('link') if x.get('name')==name);e=link.find('collision');path=ROOT/'robot_assets/ur'/e.find('geometry/mesh').get('filename').replace('package://','');m=trimesh.load(path);m.apply_transform(origin(e.find('origin')))
  robot_shapes.append(b.add_shape_mesh(body=body,mesh=newton.Mesh(m.vertices,m.faces.flatten()).compute_convex_hull(),cfg=contact))
 elif name.endswith('silicone_pad'):pass
 elif name.endswith('_pad'):
  padcfg=newton.ModelBuilder.ShapeConfig(density=0.,ke=1e6,kd=100.,mu=1.0,gap=.0001,margin=0.)
  for z in [.009375,.028125]:robot_shapes.append(b.add_shape_box(body=body,hx=.011,hy=.004,hz=.009375,xform=wp.transform(wp.vec3(0,-.0026,z),wp.quat_identity()),cfg=padcfg))

 else:
  fname=name.replace('right_','').replace('left_','');m=trimesh.load(ROOT/f'robot_assets/robotiq/robotiq_2f85/assets/{fname}.stl');m.apply_scale(.001)
  robot_shapes.append(b.add_shape_mesh(body=body,mesh=newton.Mesh(m.vertices,m.faces.flatten()).compute_convex_hull(),cfg=contact))
# Kinematic robot pairs need no contact solve. Dynamic cables retain contact
# against every robot link and both original clip components.
for i,a in enumerate(robot_shapes):
 for c in robot_shapes[i+1:]:b.shape_collision_filter_pairs.append((min(a,c),max(a,c)))
b.color(balance_colors=False);model=b.finalize(device='cuda:0')
I=np.array(b.body_inertia).reshape(-1,3,3);assert np.linalg.eigvalsh(I).min()>0
model.body_inertia.assign(I.astype(np.float32));inv_I=np.linalg.inv(I).astype(np.float32);inv_I[robot_ids]=0;model.body_inv_inertia.assign(inv_I)
enabled=model.joint_enabled.numpy();enabled[latches]=False;model.joint_enabled.assign(enabled)
pipeline=newton.CollisionPipeline(model,broad_phase='sap',rigid_contact_max=150000,contact_matching='latest')
solver=newton.solvers.SolverVBD(model,iterations=args.iterations,rigid_compliant_alm=True,rigid_contact_history=True,rigid_body_contact_buffer_size=512,friction_epsilon=1e-4,rigid_joint_linear_ke=1e7,rigid_joint_angular_ke=1e5)
solver.joint_rod_rest_kb_local.zero_();solver.joint_rod_rest_twist.zero_()
s0=model.state();s1=model.state();ctrl=model.control();contacts=pipeline.contacts();dt=1/(60*args.substeps)
robot_prev=wp.array(initial_robot,dtype=wp.transform);robot_next=wp.array(initial_robot,dtype=wp.transform);substep=wp.zeros(1,dtype=wp.int32)
@wp.kernel
def drive(q:wp.array(dtype=wp.transform),qd:wp.array(dtype=wp.spatial_vector),previous:wp.array(dtype=wp.transform),next:wp.array(dtype=wp.transform),counter:wp.array(dtype=wp.int32),offset:int,steps:int):
 j=wp.tid();u=float(counter[0]+1)/float(steps);a=previous[j];bb=next[j];pa=wp.transform_get_translation(a);pb=wp.transform_get_translation(bb);ra=wp.transform_get_rotation(a);rb=wp.transform_get_rotation(bb)
 q[offset+j]=wp.transform(pa+(pb-pa)*u,wp.quat_slerp(ra,rb,u));e=rb*wp.quat_inverse(ra)
 if e[3]<0.:e=-e
 qd[offset+j]=wp.spatial_vector((pb-pa)*60.,wp.vec3(e[0],e[1],e[2])*120.)
@wp.kernel
def preload(force:wp.array(dtype=wp.spatial_vector),ids:wp.array(dtype=wp.int32),torques:wp.array(dtype=wp.vec3)):
 j=wp.tid();force[ids[j]]+=wp.spatial_vector(wp.vec3(0.),torques[j])
@wp.kernel
def tick(counter:wp.array(dtype=wp.int32)):counter[0]+=1
topids=wp.array(tops,dtype=wp.int32);torques=wp.array([Rotation.from_euler('z',a).apply([0,.03,0]) for a in CLIP_YAWS],dtype=wp.vec3)
with wp.ScopedCapture() as cap:
 for _ in range(args.substeps):
  s0.clear_forces();wp.launch(preload,3,inputs=[s0.body_f,topids,torques]);wp.launch(drive,len(robot_ids),inputs=[s0.body_q,s0.body_qd,robot_prev,robot_next,substep,robot_start,args.substeps]);pipeline.collide(s0,contacts);solver.step(s0,s1,ctrl,contacts,dt);wp.launch(tick,1,inputs=[substep]);s0,s1=s1,s0
ids=np.array(rod_ids).reshape(COUNT,N)
def retention(q):return retained_at_clips(q,rod_ids,DS,tops)
def smooth(u):u=float(np.clip(u,0,1));return u*u*(3-2*u)
def mixrot(a,b,u):return Slerp([0,1],Rotation.from_quat([a.as_quat(),b.as_quat()]))([u])[0]
def interpolate(keys,t):
 for aa,bb in zip(keys[:-1],keys[1:]):
  if t<=bb[0]:
   u=smooth((t-aa[0])/(bb[0]-aa[0]));return (1-u)*aa[1]+u*bb[1],mixrot(aa[2],bb[2],u),(1-u)*aa[3]+u*bb[3]
 return keys[-1][1:]
queue=[['settle',-1,1.5],['connector',0,15.],['connector',1,15.],['clip',0,14.],['clip',1,14.],['clip',2,14.]]
phase_count=0;frame=0;events=[];repairs=np.zeros(3,int);wire_attempts=np.zeros((3,6),int);last_sides=np.ones((3,6),int);final_sweep=False
history={key:[] for key in ['poses','joint_positions','jaw_width','tcp_targets','tool_rotations','phases','phase_elapsed','attempt_history','latch_history','contact_counts']}
center=home.copy();toolrot=TOOL_ROT;width=.060;lastrobot=initial_robot;latch_state=[False,False];start=time.time()
if args.resume:
 d=np.load(ROOT/args.resume);meta=json.loads((ROOT/(args.resume+'.json')).read_text());assert meta['phase_boundary'], 'Resume only completed phases';frame=len(d['poses']);phase_count=meta['phase_count'];queue=meta['queue'];events=meta['events'];repairs=np.array(meta['repairs']);wire_attempts=np.array(meta['wire_attempts']);last_sides=np.array(meta['last_sides']);final_sweep=meta['final_sweep']
 for key in history:history[key]=list(d[key])
 s0.body_q.assign(d['poses'][-1]);s1.body_q.assign(d['poses'][-1]);s0.body_qd.assign(d['state_velocity']);s1.body_qd.assign(d['state_velocity']);qj=d['joint_positions'][-1].copy();center=d['tcp_targets'][-1].copy();toolrot=Rotation.from_quat(d['tool_rotations'][-1]);width=float(d['jaw_width'][-1]);lastrobot=d['poses'][-1,robot_ids].copy();robot_prev.assign(lastrobot);robot_next.assign(lastrobot);latch_state=d['latch_history'][-1].tolist();enabled[latches]=latch_state;model.joint_enabled.assign(enabled)
 events.append(dict(event='resume_at_phase_boundary',time=frame/60,source=args.resume,preserved_positions_and_velocities=True))
def save(stem,boundary=False):
 target=ROOT/(stem+'.npz');temp=ROOT/(stem+'.tmp.npz');np.savez_compressed(temp,**history,state_velocity=s0.body_qd.numpy(),fps=60,segments=N,segment_length=DS,clip_top_bodies=tops,robot_start=robot_start,robot_names=robot_names,plug_bodies=plugs,plug_anchors=plug_anchors,anchor_q=anchor_q,rod_ids=rod_ids);os.replace(temp,target)
 meta=dict(phase_count=phase_count,queue=queue,events=events,repairs=repairs.tolist(),wire_attempts=wire_attempts.tolist(),last_sides=last_sides.tolist(),final_sweep=final_sweep,phase_boundary=boundary)
 (ROOT/(stem+'.npz.json')).write_text(json.dumps(meta,indent=2)+'\n')

def choose_repair(q,k):
 missing=np.flatnonzero(~np.array(retention(q)[k]));cr=Rotation.from_euler('z',CLIP_YAWS[k]);local=cr.inv().apply(q[ids,:3].reshape(-1,3)-CLIP_CENTERS[k]).reshape(COUNT,N,3);options=[]
 for c in missing:
  if wire_attempts[k,c]>=2:continue
  side=-int(last_sides[k,c]);candidates=np.flatnonzero((side*local[c,:,1]>.027)&(side*local[c,:,1]<.065))
  if not len(candidates):candidates=np.array([np.argmin(abs(local[c,:,1]-side*.045))])
  # Favor the protruding part on the opposite side, with clearance from neighbors.
  for seg in candidates:
   pt=local[c,seg];others=np.delete(local,c,axis=0).reshape(-1,3);separation=float(np.linalg.norm(others-pt,axis=1).min());score=float(pt[0]+min(separation,.010)*.5-.15*abs(abs(pt[1])-.045));options.append((score,int(c),int(seg),side,separation))
 if not options:return None
 _,c,seg,side,sep=max(options);return c,seg,side,sep

def enqueue_recovery(q,k):
 if all(retention(q)[k]):return False
 choice=choose_repair(q,k)
 if repairs[k]>=args.max_repairs_per_clip or choice is None:
  events.append(dict(event='recovery_limit_reached',time=frame/60,clip=k+1,retained=retention(q)[k]));return False
 queue.insert(0,['repair',k,12.]);return True

while queue:
 phase,k,duration=queue.pop(0);phase_start=center.copy();phase_rot=toolrot;phase_width=width;keys=None;current=s0.body_q.numpy();grasp_offset=np.array([0,0,.005]);calibrated_lift=False;calibrated_socket=False;recovery_seated_frames=0;recovery_hold=None
 if phase=='connector' and k==1 and not latch_state[0]:events.append(dict(event='stopped_first_connector_not_seated',time=frame/60));break
 if phase in ['clip','repair'] and not all(latch_state):events.append(dict(event='stopped_connectors_not_seated',time=frame/60));break
 if phase=='clip':
  cr=Rotation.from_euler('z',CLIP_YAWS[k]);localq=cr.inv().apply(current[ids,:3].reshape(-1,3)-CLIP_CENTERS[k]).reshape(COUNT,N,3);grasp_ids=np.argmin(abs(localq[:,:,1]-.045),axis=1);last_sides[k,:]=1
 if phase=='repair':
  choice=choose_repair(current,k)
  if choice is None:continue
  cable,segment,side,separation=choice;repairs[k]+=1;wire_attempts[k,cable]+=1;last_sides[k,cable]=side
  events.append(dict(event='opposite_side_wire_selection',time=frame/60,clip=k+1,cable=cable+1,segment=segment,side=side,separation_mm=separation*1000,retained_before=retention(current)[k]));print('RECOVERY',events[-1],flush=True)
 print('PHASE',phase_count,phase,k+1,'time',frame/60,flush=True)
 for pf in range(round(duration*60)):
  local=pf/60;t=frame/60;current=s0.body_q.numpy()
  if phase=='settle':center=home.copy();toolrot=TOOL_ROT;width=.060
  elif phase=='connector':
   bp=current[plugs[k],:3];br=Rotation.from_quat(current[plugs[k],3:]);tracked=bp+br.apply(grasp_offset)
   # Use a negative yaw winding for the far connector so the return can unwind safely.
   gr=br*TOOL_ROT
   if local<=2.7:
    if local<=1.3:
     u=smooth(local/1.3);center=(1-u)*phase_start+u*(tracked+[0,0,.08]);toolrot=mixrot(phase_rot,gr,u);width=(1-u)*phase_width+u*.060
    else:
     u=smooth((local-1.3)/1.4);center=tracked+[0,0,.08*(1-u)];toolrot=gr;width=.060
    pickup=center.copy();pickuprot=toolrot
   else:
    sr=Rotation.from_euler('z',SOCKET_YAWS[k]);angle=math.radians(25)*(1-smooth((local-8.6)/1.8));desired_rot=sr*Rotation.from_euler('x',angle);desired_pos=SOCKET_POSITIONS[k]+sr.apply([0,.0135*(math.cos(angle)-1)+.004*math.sin(angle),.0135*math.sin(angle)+.004*(math.cos(angle)-1)])
    if keys is None:keys=[(2.7,pickup,pickuprot,.060),(3.6,pickup,pickuprot,.0468),(5.,pickup+[0,0,.075],pickuprot,.0468)]
    if local<5:center,toolrot,width=interpolate(keys,local)
    else:
     if not calibrated_lift or (local>=7 and not calibrated_socket):
      relpos=toolrot.inv().apply(bp-center);relrot=toolrot.inv()*br
      def held_target(pos,rot):
       tr=rot*relrot.inv();return pos-tr.apply(relpos),tr
      target,rotation=held_target(desired_pos,desired_rot)
      keys=[(local,center.copy(),toolrot,.0468),(7.,target+[0,0,.065],rotation,.0468)] if local<7 else [(7.,center.copy(),toolrot,.0468),(8.6,target,rotation,.0468)]
      calibrated_lift=True;calibrated_socket=local>=7;events.append(dict(event='in_hand_calibration',time=t,connector=k+1,relative_position=relpos.tolist()))
     if local<8.6:center,toolrot,width=interpolate(keys,local)
     elif local<11:center,toolrot=held_target(desired_pos,desired_rot);width=.0468
     else:
      if pf==660:
       seated_center=center.copy();seated_rot=toolrot;keys=[(11.,center.copy(),toolrot,.0468),(11.9,center.copy(),toolrot,.070),(13.2,center+[0,0,.10],toolrot,.070),(15.,home,TOOL_ROT,.060)]
      center,toolrot,width=interpolate(keys,local)
      if k==1 and local>=13.2:
       u=smooth((local-13.2)/1.8);yaw=(seated_rot*TOOL_ROT.inv()).as_euler('xyz')[2]
       if yaw>0:yaw-=2*np.pi
       residual=Rotation.from_euler('z',yaw).inv()*seated_rot*TOOL_ROT.inv();toolrot=Rotation.from_euler('z',yaw*(1-u))*mixrot(residual,Rotation.identity(),u)*TOOL_ROT
    if 10.4<=local<15 and not latch_state[k]:
     err=np.linalg.norm(bp-SOCKET_POSITIONS[k]);ang=(sr.inv()*br).magnitude()
     if err<.0015 and ang<math.radians(6):
      enabled[latches[k]]=True;model.joint_enabled.assign(enabled);latch_state[k]=True;events.append(dict(event='connector_latched',time=t,connector=k+1,position_error_mm=err*1000,angle_error_deg=math.degrees(ang)));print('LATCH',events[-1],flush=True)
  elif phase in ['clip','repair']:
   cr=Rotation.from_euler('z',CLIP_YAWS[k]);gr=cr*TOOL_ROT
   if phase=='clip':
    pts=current[ids[np.arange(COUNT),grasp_ids],:3];pp=cr.inv().apply(pts-CLIP_CENTERS[k]);tracked=CLIP_CENTERS[k]+cr.apply([(pp[:,0].min()+pp[:,0].max())/2,pp[:,1].mean(),pp[:,2].min()+.0002]);opening=float(np.clip(np.ptp(pp[:,0])+.010,.025,.080));hold=float(np.clip(np.ptp(pp[:,0])+.0016,.003,.020))
   else:
    tracked=current[ids[cable,segment],:3]+[0,0,.0002];opening=.012;hold=.0018
   tracked[2]=max(.0022,tracked[2])
   if local<=2.7:
    if local<=1.3:
     u=smooth(local/1.3);center=(1-u)*phase_start+u*(tracked+[0,0,.075]);toolrot=mixrot(phase_rot,gr,u);width=(1-u)*phase_width+u*opening
    else:
     u=smooth((local-1.3)/1.4);center=tracked+[0,0,.075*(1-u)];toolrot=gr;width=opening
    pickup=center.copy();grasp_width=hold;open_width=opening
   else:
    if keys is None:
     if phase=='clip':
      outside=CLIP_CENTERS[k]+cr.apply([.047,.045,.006]);inside=CLIP_CENTERS[k]+cr.apply([-.013,.045,.002]);keys=[(2.7,pickup,gr,open_width),(3.6,pickup,gr,grasp_width),(4.6,pickup+[0,0,.024],gr,grasp_width),(6.,outside,gr,grasp_width),(8.2,inside,gr,grasp_width),(10.,inside,gr,grasp_width),(10.8,inside,gr,.080),(12.,inside+[0,0,.09],gr,.080),(14.,home,TOOL_ROT,.060)]
     else:
      # Maintain the chosen material point on the opposite longitudinal side.
      py=float(cr.inv().apply(pickup-CLIP_CENTERS[k])[1]);inside=CLIP_CENTERS[k]+cr.apply([-.020,py,.001]);keys=[(2.7,pickup,gr,open_width),(3.6,pickup,gr,.0018),(4.3,pickup+[0,0,.004],gr,.0018),(7.5,inside,gr,.0018),(8.8,inside,gr,.0018),(9.5,inside,gr,.060),(10.5,inside+[0,0,.08],gr,.060),(12.,home,TOOL_ROT,.060)]
    center,toolrot,width=interpolate(keys,local)
    if phase=='repair' and 4.3<=local<9.5:
     recovery_seated_frames=recovery_seated_frames+1 if retention(current)[k][cable] else 0
     if recovery_hold is None and recovery_seated_frames>=9:
      recovery_hold=center.copy();events.append(dict(event='opposite_side_wire_seated_during_pull',time=t,clip=k+1,cable=cable+1,side=side));print('WIRE SEATED',events[-1],flush=True)
      for ki,key in enumerate(keys):
       if key[0] in [7.5,8.8,9.5]:keys[ki]=(key[0],recovery_hold.copy(),key[2],key[3])
       elif key[0]==10.5:keys[ki]=(key[0],recovery_hold+[0,0,.08],key[2],key[3])
     if recovery_hold is not None:center=recovery_hold.copy()
  theta=theta_for_gap(width);qj=ik(goal_flange(center,theta,toolrot),qj);nextrobot=packed(robot_poses(qj,theta));robot_prev.assign(lastrobot);robot_next.assign(nextrobot);substep.zero_();wp.capture_launch(cap.graph);lastrobot=nextrobot
  q=s0.body_q.numpy().copy();assert np.isfinite(q).all()
  for key,value in dict(poses=q,joint_positions=qj.copy(),jaw_width=width,tcp_targets=center.copy(),tool_rotations=toolrot.as_quat(),phases=f'{phase}_{k+1}',phase_elapsed=local,attempt_history=repairs[k] if phase=='repair' else 1,latch_history=latch_state.copy(),contact_counts=int(contacts.rigid_contact_count.numpy()[0])).items():history[key].append(value)
  frame+=1
  if frame%60==0:print('FRAME',frame,'time',frame/60,'counts',np.sum(retention(q),axis=1).tolist(),'latches',latch_state,'wall',round(time.time()-start,1),flush=True)
  if frame%300==0:save(args.output+'_live')
 phase_count+=1;q=s0.body_q.numpy()
 if phase in ['clip','repair']:
  events.append(dict(event='post_insertion_retention_check',time=frame/60,clip=k+1,phase=phase,retained=retention(q)));enqueue_recovery(q,k)
 if not queue and not final_sweep:
  final_sweep=True
  for ck in reversed(range(3)):enqueue_recovery(q,ck)
  if not queue:queue.append(['settle',-1,2.])
 save(args.output+'_boundary',True)
 if args.stop_after and phase_count>=args.stop_after:break
save(args.output,True)
report=dict(cable_count=COUNT,length_mm=L*1000,diameter_mm=RADIUS*2000,segments=N,solver='Newton SolverVBD compliant ALM',iterations=args.iterations,physics_hz=60*args.substeps,EI_Nm2=EI,grasp='Contact-only pads; no support lips or attachments',connector_modification='Handling rib removed; original side tabs and ridges collide',latches='Idealized socket locks activate only within 1.5 mm and 6 degrees; used to retain connectors against cable loads',events=events,final_latched=latch_state,final_retained=np.sum(retention(s0.body_q.numpy()),axis=1).tolist(),seconds=frame/60,runtime_seconds=time.time()-start,complete=not queue,repairs=repairs.tolist(),pose_servo=False)
(ROOT/(args.output+'.json')).write_text(json.dumps(report,indent=2)+'\n');print('DONE',report['final_retained'],flush=True)
