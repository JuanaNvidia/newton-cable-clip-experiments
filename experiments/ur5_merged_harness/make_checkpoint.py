"""Restore the exact 143.5 s branch checkpoint without rerunning earlier physics."""
from pathlib import Path
import argparse,json,hashlib,numpy as np
from motion_io import load_motion
root=Path(__file__).resolve().parent;p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists() or Path(str(a.output)+'.json').exists():p.error('Choose a new output path.')
spec=json.loads((root/'restart_143_5.json').read_text());state_path=root/'restart_143_5_state.npz';assert hashlib.sha256(state_path.read_bytes()).hexdigest()==spec['state_sha256'];state=np.load(state_path);record=load_motion(root/'motion.npz');data={}
for key in spec['frame_arrays']:
 value=record[key][:spec['prefix_frames']];assert hashlib.sha256(value.tobytes()).hexdigest()==spec['prefix_array_sha256'][key],key;data[key]=value
for key in state.files:data[key]=state[key]
np.savez_compressed(a.output,**data);Path(str(a.output)+'.json').write_text(json.dumps(spec['controller_state'],indent=2)+'\n');print('Restored phase-boundary poses, velocities and controller:',a.output)
