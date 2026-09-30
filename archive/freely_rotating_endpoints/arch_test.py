"""Gravity + force deflection test. Straight rest rods; pinned endpoints, free middle."""
from pathlib import Path
import json, math, argparse
import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation
import warp as wp
import newton
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=8);a=p.parse_args()
wp.init();wp.set_device('cuda:0')
N=61;L=.3048;ds=L/N;r=.001;span=.24
# Equal-length chords on a circular arc. This is an initial deformed state,
# not rest curvature: every rod joint has identity local anchor rotations.
angle=brentq(lambda t: ds*math.sin(t/2)/math.sin(t/(2*N))-span,.01,5.)
rad=ds/(2*math.sin(angle/(2*N)))
theta=np.linspace(-angle/2,angle/2,N+1)
y=rad*np.sin(theta);z=.015+rad*(np.cos(theta)-math.cos(angle/2))
b=newton.ModelBuilder(gravity=(0.,0.,-9.81))
# Correct capsule-overlap mass to the solid-cylinder line density.
density=1000*ds/(ds+4*r/3)
cfg=newton.ModelBuilder.ShapeConfig(density=density,ke=1e6,kd=100.,mu=.5,gap=.0002)
b.add_shape_box(body=-1,hx=.10,hy=.19,hz=.005,xform=wp.transform(wp.vec3(0,0,-.005),wp.quat_identity()),cfg=newton.ModelBuilder.ShapeConfig(density=0.))
base_EI=.002*180/math.pi*.005
factors=[.1,1.,2.];chains=[]
for c,factor in enumerate(factors):
    chain=[];joints=[]
    for i in range(N):
        v=np.array([0.,y[i+1]-y[i],z[i+1]-z[i]])/ds
        rot=Rotation.from_euler('x',math.atan2(-v[1],v[2])).as_quat()
        body=b.add_link(xform=wp.transform(wp.vec3((c-1)*.045,(y[i]+y[i+1])/2,(z[i]+z[i+1])/2),wp.quat(*rot)))
        chain.append(body);b.add_shape_capsule(body=body,radius=r,half_height=ds/2,cfg=cfg)
        if i:joints.append(b.add_joint_rod(parent=chain[-2],child=body,parent_xform=wp.transform(wp.vec3(0,0,ds/2),wp.quat_identity()),child_xform=wp.transform(wp.vec3(0,0,-ds/2),wp.quat_identity()),stretch_stiffness=1e6,stretch_damping=.1,bend_stiffness=base_EI*factor/ds,bend_damping=.00572957795*.005/ds))
    b.add_articulation(joints)
    for i,sgn in [(0,-1),(N-1,1)]:
        j=b.add_joint_ball(parent=-1,child=chain[i],parent_xform=wp.transform(wp.vec3((c-1)*.045,sgn*span/2,.015),wp.quat_identity()),child_xform=wp.transform(wp.vec3(0,0,sgn*ds/2),wp.quat_identity()))
        b.add_articulation([j])
    chains.append(chain)
b.color();model=b.finalize(device='cuda:0')
I=np.array(b.body_inertia).reshape(-1,3,3);assert np.linalg.eigvalsh(I).min()>0
model.body_inertia.assign(I.astype(np.float32));model.body_inv_inertia.assign(np.linalg.inv(I).astype(np.float32))
pipeline=newton.CollisionPipeline(model,broad_phase='sap',rigid_contact_max=10000,contact_matching='latest')
solver=newton.solvers.SolverVBD(model,iterations=40,rigid_compliant_alm=True,rigid_contact_history=True,rigid_body_contact_buffer_size=128)
s0=model.state();s1=model.state();ctrl=model.control();contacts=pipeline.contacts();clock=wp.zeros(1,dtype=float);dt=1/3840
@wp.kernel
def force(f:wp.array(dtype=wp.spatial_vector),clock:wp.array(dtype=float)):
    c=wp.tid();t=clock[0]
    if t>3.0 and t<4.0:
        f[c*61+30]=wp.spatial_vector(wp.vec3(0.,0.,-0.02*wp.sin(3.14159265*(t-3.0))),wp.vec3(0.))
@wp.kernel
def tick(clock:wp.array(dtype=float),dt:float):clock[0]+=dt
with wp.ScopedCapture() as cap:
    for _ in range(64):
        s0.clear_forces();wp.launch(force,3,inputs=[s0.body_f,clock]);pipeline.collide(s0,contacts);solver.step(s0,s1,ctrl,contacts,dt);wp.launch(tick,1,inputs=[clock,dt]);s0,s1=s1,s0
poses=[]
for frame in range(round(a.seconds*60)):
    wp.capture_launch(cap.graph);q=s0.body_q.numpy();assert np.isfinite(q).all();poses.append(q.copy())
    if frame%60==0:print('FRAME',frame,'apex_mm',q[np.array(chains)[:,N//2],2]*1000,flush=True)
poses=np.array(poses);np.savez_compressed(ROOT/'arch_motion.npz',poses=poses,fps=60,segments=N,segment_length=ds,factors=factors)
results=[]
for c,factor in enumerate(factors):
    q=poses[:,c*N:(c+1)*N];rot=Rotation.from_quat(q[:,:,3:].reshape(-1,4));vec=rot.apply(np.tile([0,0,ds/2],(len(q)*N,1))).reshape(len(q),N,3)
    ends0=q[:,:,:3]-vec;ends1=q[:,:,:3]+vec
    results.append({'factor':factor,'EI_Nm2':base_EI*factor,'bend_stiffness_Nm_per_rad':base_EI*factor/ds,'mass_g':float(model.body_mass.numpy()[chains[c]].sum()*1000),'apex_at_3s_mm':float(q[179,:,2].max()*1000),'min_apex_during_press_mm':float(q[180:240,:,2].max(axis=1).min()*1000),'final_apex_mm':float(q[-1,:,2].max()*1000),'last_second_apex_range_mm':float(np.ptp(q[-60:,:,2].max(axis=1))*1000),'max_joint_gap_mm':float(np.linalg.norm(ends1[:,:-1]-ends0[:,1:],axis=-1).max()*1000),'max_end_anchor_error_mm':float(max(np.linalg.norm(ends0[:,0]-[(c-1)*.045,-span/2,.015],axis=1).max(),np.linalg.norm(ends1[:,-1]-[(c-1)*.045,span/2,.015],axis=1).max())*1000)})
report={'length_m':L,'diameter_m':2*r,'endpoint_span_m':span,'endpoint_elevation_m':.015,'boundary_conditions':'Ball-joint pinned positions, freely rotating ends; straight rest rods initially bent upward. No midpoint constraints.','temporary_load':'0.02 N downward half-sine pulse at midpoint from 3 to 4 seconds','solver':'Newton 1.6 SolverVBD, 40 iterations, 3840 Hz','results':results}
(ROOT/'arch_results.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
