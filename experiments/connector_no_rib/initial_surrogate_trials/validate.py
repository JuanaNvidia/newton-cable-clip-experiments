"""Recorded pose checks, joint/FK checks and explicit sampled collision audits."""
from pathlib import Path
import argparse,json,hashlib,subprocess,sys
import numpy as np
from scipy.spatial.transform import Rotation
from robot import poses,packed,theta_for_gap
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--motion',default='flat');a=p.parse_args();path=ROOT/(a.motion+'.npz');d=np.load(path);q=d['poses'];j=d['joint_positions'];errs=[]
for i in range(0,len(q),6):
 pred=packed(poses(j[i],theta_for_gap(float(d['jaw_width'][i]))));errs.append(np.max(np.linalg.norm(q[i,1:,:3]-pred[:,:3],axis=1)))
checks=dict(finite=bool(np.isfinite(q).all()),joint_bounds=bool(np.max(abs(j))<=2*np.pi+1e-6),joint_speed=bool(np.max(abs(np.diff(j,axis=0)*60))<=np.pi),robot_FK=bool(max(errs)<1e-5))
e=np.linalg.norm(q[-120:,0,:3]-[-.025,.16,.0305],axis=1);r=Rotation.from_quat(q[-120:,0,3:]).magnitude();checks['seated_last_two_seconds']=bool(e.max()<.0015 and r.max()<np.radians(6))
for script,key in [('check_connector_contacts.py','connector_contacts'),('check_robot_clearance.py','robot_static_clearance')]:
 result=subprocess.run([sys.executable,str(ROOT/script),'--motion',a.motion]);checks[key]=result.returncode==0
report=dict(motion_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),passed=all(checks.values()),checks=checks,final_position_error_mm=float(e.max()*1000),final_angle_error_deg=float(np.degrees(r).max()),max_robot_fk_error_m=float(max(errs)),max_joint_speed_rad_s=float(np.max(abs(np.diff(j,axis=0)*60))),limitations=['Sampled geometry checks, not swept-volume proof','Kinematic UR5; no force or torque limits, no self-collision audit','No attached cables; no latch mechanism','Existing fingertip support lips retained'])
(ROOT/(a.motion+'_validation.json')).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));sys.exit(0 if report['passed'] else 1)
