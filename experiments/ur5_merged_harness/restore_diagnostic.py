"""Restore an exact diagnostic branch using the shared selected-recording prefix."""
from pathlib import Path
import argparse,hashlib,json,numpy as np
from motion_io import load_motion
root=Path(__file__).resolve().parent;p=argparse.ArgumentParser();p.add_argument('name',choices=['failed_loose_bundle','failed_tighter_bundle','narrow_probe','failed_wide_release']);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():p.error('Choose a new output path.')
spec=json.loads((root/'diagnostics'/(a.name+'_tail.json')).read_text());tail=np.load(root/'diagnostics'/(a.name+'_tail.npz'));base=load_motion(root/'motion.npz');n=spec['prefix_frames'];result={}
for key in tail.files:
 value=tail[key]
 if key in spec['frame_arrays']:
  prefix=base[key][:n];assert hashlib.sha256(prefix.tobytes()).hexdigest()==spec['prefix_array_sha256'][key];value=np.concatenate([prefix,value])
 expected=spec['arrays'][key];assert list(value.shape)==expected['shape'] and str(value.dtype)==expected['dtype'] and hashlib.sha256(value.tobytes()).hexdigest()==expected['sha256'],key
 result[key]=value
if 'controller_state' in spec:Path(str(a.output)+'.json').write_text(json.dumps(spec['controller_state'],indent=2)+'\n')
np.savez_compressed(a.output,**result);a.output.with_suffix('.json').write_text(json.dumps(spec['run_metadata'],indent=2)+'\n');print('Restored exact arrays:',a.output)
