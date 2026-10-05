"""Contact-only body grasp, release above socket, optional press; no latch constraint."""
import argparse,json,time
from model import *
from scipy.spatial.transform import Slerp
p=argparse.ArgumentParser();p.add_argument('--strategy',choices=['flat','angled'],default='flat');p.add_argument('--output');p.add_argument('--grasp-z',type=float,default=-.003);p.add_argument('--grasp-gap',type=float,default=.0438);p.add_argument('--iterations',type=int,default=40);p.add_argument('--substeps',type=int,default=32);a=p.parse_args()
b.color(balance_colors=False);model=b.finalize(device='cuda:0')
pipeline=newton.CollisionPipeline(model,broad_phase='sap',rigid_contact_max=10000,contact_matching='latest')
solver=newton.solvers.SolverVBD(model,iterations=a.iterations,rigid_compliant_alm=True,rigid_contact_history=True,rigid_body_contact_buffer_size=256,friction_epsilon=1e-4)
s0=model.state();s1=model.state();ctrl=model.control();contacts=pipeline.contacts()
prev=wp.array(initial_robot,dtype=wp.transform);nxt=wp.array(initial_robot,dtype=wp.transform);counter=wp.zeros(1,dtype=wp.int32)
@wp.kernel
def drive(q:wp.array(dtype=wp.transform),qd:wp.array(dtype=wp.spatial_vector),previous:wp.array(dtype=wp.transform),next:wp.array(dtype=wp.transform),counter:wp.array(dtype=wp.int32),offset:int,steps:int):
 j=wp.tid();u=float(counter[0]+1)/float(steps);aa=previous[j];bb=next[j];pa=wp.transform_get_translation(aa);pb=wp.transform_get_translation(bb);ra=wp.transform_get_rotation(aa);rb=wp.transform_get_rotation(bb)
 q[offset+j]=wp.transform(pa+(pb-pa)*u,wp.quat_slerp(ra,rb,u));e=rb*wp.quat_inverse(ra)
 if e[3]<0.:e=-e
 qd[offset+j]=wp.spatial_vector((pb-pa)*60.,wp.vec3(e[0],e[1],e[2])*120.)
@wp.kernel
def tick(counter:wp.array(dtype=wp.int32)):counter[0]+=1
with wp.ScopedCapture() as cap:
 for _ in range(a.substeps):
  s0.clear_forces();wp.launch(drive,len(robot_ids),inputs=[s0.body_q,s0.body_qd,prev,nxt,counter,robot_start,a.substeps]);pipeline.collide(s0,contacts);solver.step(s0,s1,ctrl,contacts,1/(60*a.substeps));wp.launch(tick,1,inputs=[counter]);s0,s1=s1,s0

def smooth(x):x=np.clip(x,0,1);return x*x*(3-2*x)
def mixrot(x,y,u):return Slerp([0,1],Rotation.from_quat([x.as_quat(),y.as_quat()]))([u])[0]
def interpolate(keys,t):
 for aa,bb in zip(keys[:-1],keys[1:]):
  if t<=bb[0]:
   u=smooth((t-aa[0])/(bb[0]-aa[0]));return (1-u)*aa[1]+u*bb[1],mixrot(aa[2],bb[2],u),(1-u)*aa[3]+u*bb[3]
 return keys[-1][1:]
center=home.copy();rot=TOOL_ROT;width=.06;lastrobot=initial_robot;history=[];jh=[];wh=[];th=[];rh=[];ph=[];events=[];start=time.time();offset=np.array([0,0,a.grasp_z]);keys=None
for frame in range(1200):
 t=frame/60;body=s0.body_q.numpy()[plug];bp=body[:3];br=Rotation.from_quat(body[3:]);phase='approach'
 if t<3:
  target=bp+br.apply(offset);u=smooth(t/1.5) if t<1.5 else smooth((t-1.5)/1.5)
  center=(1-u)*home+u*(target+[0,0,.07]) if t<1.5 else target+[0,0,.07*(1-u)];rot=br*TOOL_ROT;width=.060;pickup=center.copy();pickuprot=rot
 elif t<5:
  if keys is None:keys=[(3,pickup,pickuprot,.060),(3.8,pickup,pickuprot,a.grasp_gap),(5,pickup+[0,0,.075],pickuprot,a.grasp_gap)]
  center,rot,width=interpolate(keys,t);phase='body pinch and lift'
 else:
  if frame==300:
   relpos=rot.inv().apply(bp-center);relrot=rot.inv()*br
   tilt=Rotation.from_euler('x',25 if a.strategy=='angled' else 0,degrees=True)
   target=SOCKET+[0,0,.019 if a.strategy=='angled' else .014]
   release_rot=tilt*relrot.inv();release_center=target-release_rot.apply(relpos)
   keys=[(5,center.copy(),rot,a.grasp_gap),(7,release_center+[0,0,.05],release_rot,a.grasp_gap),(8.5,release_center,release_rot,a.grasp_gap),(9.3,release_center,release_rot,.075),(10.5,release_center+[0,0,.08],release_rot,.075),(12,SOCKET+[0,-.004,.09],TOOL_ROT,.010)]
   events.append(dict(event='in_hand_calibration',time=t,plug_position=bp.tolist(),relative_position=relpos.tolist()))
  if t<12:center,rot,width=interpolate(keys,t);phase='transport' if t<7 else 'lower above guides' if t<8.5 else 'release into socket' if t<9.3 else 'withdraw and observe'
  else:
   if frame==720:
    # Press only after fingers have opened, retracted, and closed in free space.
    press=SOCKET+np.array([0,-.004,.0052]);keys=[(12,center.copy(),rot,.010),(13.5,press,TOOL_ROT,.010),(15,press,TOOL_ROT,.010),(17,press+[0,0,.10],TOOL_ROT,.010),(20,home,TOOL_ROT,.060)]
   center,rot,width=interpolate(keys,t);phase='press original housing' if t<15 else 'withdraw; gravity retention'
 theta=theta_for_gap(width);qj=ik(goal_flange(center,theta,rot),qj);now=packed(robot_poses(qj,theta));prev.assign(lastrobot);nxt.assign(now);counter.zero_();wp.capture_launch(cap.graph);lastrobot=now
 history.append(s0.body_q.numpy());jh.append(qj.copy());wh.append(width);th.append(center.copy());rh.append(rot.as_quat());ph.append(phase)
 if frame%120==0:print(a.strategy,round(t,2),phase,'plug',history[-1][0,:3], 'wall',round(time.time()-start,1),flush=True)
q=np.asarray(history);err=np.linalg.norm(q[-120:,plug,:3]-SOCKET,axis=1);ang=Rotation.from_quat(q[-120:,plug,3:]).magnitude()*180/np.pi
out=ROOT/(a.output or a.strategy)
np.savez_compressed(str(out)+'.npz',poses=q,joint_positions=jh,jaw_width=wh,tcp_targets=th,tool_rotations=rh,phases=ph,robot_names=robot_names,robot_start=robot_start,plug_bodies=[plug],fps=60)
report=dict(strategy=a.strategy,grasp_gap_m=a.grasp_gap,grasp_height_m=a.grasp_z,solver='Newton SolverVBD',iterations=a.iterations,substeps=a.substeps,physics_hz=60*a.substeps,connector_mass_kg=.015,contact_only_grasp=True,latch_constraint=False,cables_included=False,handling_rib=False,existing_fingertip_support_lips=True,final_position_error_mm=float(err.max()*1000),final_rotation_error_deg=float(ang.max()),seated=bool(err.max()<.0015 and ang.max()<6),max_joint_speed_rad_s=float(np.max(np.abs(np.diff(jh,axis=0)*60))),events=events)
Path(str(out)+'.json').write_text(json.dumps(report,indent=2)+'\n');print(report,flush=True)
