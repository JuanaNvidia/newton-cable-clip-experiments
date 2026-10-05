"""Rebuild a task-boundary restart from the lossless recording and saved velocities."""
from pathlib import Path
import argparse,numpy as np
from motion_io import load_motion
root=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--frame',required=True,type=int);p.add_argument('--output',required=True);a=p.parse_args()
d=load_motion(root/'motion.npz');states=np.load(root/'restart_states.npz');matches=np.where(states['frames']==a.frame)[0];assert len(matches)==1,'Choose a saved task boundary'
k=int(matches[0]);assert np.array_equal(d['poses'][a.frame-1],states['body_q'][k]);frame_arrays={'poses','joint_positions','jaw_width','tcp_targets','tool_rotations','phases','phase_elapsed','attempt_history','latch_history','contact_counts'}
keys=d.files if hasattr(d,'files') else list(d)
result={key:d[key][:a.frame] if key in frame_arrays else d[key] for key in keys};result['state_velocity']=states['body_qd'][k]
path=root/a.output;assert not path.exists(),'Choose a fresh output name';np.savez_compressed(path,**result);print(path)
