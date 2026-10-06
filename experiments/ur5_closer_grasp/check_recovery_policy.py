"""Verify missed-wire selections use actual retention and alternate longitudinal sides."""
from pathlib import Path
import argparse,json
import numpy as np
from scipy.spatial.transform import Rotation as R
from motion_io import load_motion,motion_sha256
from audit_geometry import retained_at_clips
from scene import CLIP_CENTERS,CLIP_YAWS
root=Path(__file__).resolve().parent;p=argparse.ArgumentParser();p.add_argument('--motion',default='motion');a=p.parse_args();d=load_motion(root/(a.motion+'.npz'));meta=json.loads((root/(a.motion+'.json')).read_text());q=d['poses'];ids=d['rod_ids'].reshape(6,-1);last=np.ones((3,6),int);checks=[];observations=[];events=meta['events']
for e in events:
 if e['event']=='post_insertion_retention_check' and e['phase']=='clip':last[e['clip']-1,:]=1
 if e['event']!='opposite_side_wire_selection':continue
 k=e['clip']-1;c=e['cable']-1;frame=round(e['time']*60);before=q[max(0,frame-1)];mask=retained_at_clips(before,d['rod_ids'],float(d['segment_length']),d['clip_top_bodies'])[k];point=R.from_euler('z',CLIP_YAWS[k]).inv().apply(before[ids[c,e['segment']],:3]-CLIP_CENTERS[k]);selected_missing=not mask[c];opposite=e['side']==-last[k,c];side_matches=e['side']*point[1]>0;last[k,c]=e['side']
 checks.extend([selected_missing,opposite,side_matches]);names=list(d['robot_names']);base=int(d['robot_start']);touch=[False,False]
 for f in range(frame+216,min(frame+259,len(q)),6):
  rods=q[f,ids[c]];v=R.from_quat(rods[:,3:]).apply(np.tile([0,0,float(d['segment_length'])/2],(len(rods),1)));pts=np.concatenate([rods[:,:3]-v,rods[:,:3],rods[:,:3]+v])
  for s,side in enumerate(['left','right']):
   pad=q[f,base+names.index(side+'_pad')];local=R.from_quat(pad[3:]).inv().apply(pts-pad[:3]);delta=abs(local-[0,-.0026,.01875])-[.011,.004,.01875];distance=np.linalg.norm(np.maximum(delta,0),axis=1)+np.minimum(delta.max(axis=1),0);touch[s]|=bool(distance.min()<.00135)
 observations.append(dict(time=e['time'],clip=k+1,cable=c+1,selected_missing=selected_missing,opposite_side=bool(opposite),side_matches_position=bool(side_matches),selected_wire_near_each_pad=touch))
# A missed target clip must trigger an immediate attempt or a logged limit.
for e in events:
 if e['event']!='post_insertion_retention_check' or all(e['retained'][e['clip']-1]):continue
 checks.append(any(z['event'] in ['opposite_side_wire_selection','recovery_limit_reached'] and z.get('clip')==e['clip'] and abs(z['time']-e['time'])<1e-8 for z in events))
r=dict(motion_sha256=motion_sha256(root/(a.motion+'.npz')),passed=all(checks),checks_count=len(checks),recovery_attempts=observations,limits=[e for e in events if e['event']=='recovery_limit_reached'],note='Pad proximity is a diagnostic, not force sensing or proof of grasp security. Final channel occupancy is checked separately and does not require the lid to close.')
(root/('recovery_validation.json' if a.motion=='motion' else a.motion+'_recovery_validation.json')).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
