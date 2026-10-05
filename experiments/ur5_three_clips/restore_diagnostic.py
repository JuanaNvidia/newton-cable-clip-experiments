"""Reconstruct the failed lower-entry recording without duplicating its shared prefix."""
from pathlib import Path
import argparse,json,hashlib,numpy as np
from motion_io import load_motion
root=Path(__file__).resolve().parent;p=argparse.ArgumentParser();p.add_argument('--name',choices=['lower_entry','angled_bundle'],default='lower_entry');p.add_argument('--output');a=p.parse_args();path=root/(a.output or a.name+'_failure.npz');assert not path.exists() and not path.with_suffix('.json').exists(),'Choose an unused output name'
manifest=json.loads((root/f'diagnostics/{a.name}_tail.json').read_text());tail=np.load(root/f'diagnostics/{a.name}_tail.npz');parent=load_motion(root/'motion.npz');n=manifest['prefix_frames'];result={}
for key in tail.files:
 if key in manifest['frame_arrays']:
  prefix=parent[key][:n];assert hashlib.sha256(prefix.tobytes()).hexdigest()==manifest['prefix_array_sha256'][key],key;value=np.concatenate([prefix,tail[key]])
 else:value=tail[key]
 expected=manifest['arrays'][key];assert list(value.shape)==expected['shape'] and str(value.dtype)==expected['dtype'];assert hashlib.sha256(value.tobytes()).hexdigest()==expected['sha256'],key;result[key]=value
np.savez_compressed(path,**result);path.with_suffix('.json').write_text(json.dumps(manifest['run_metadata'],indent=2)+'\n');print('Reconstructed and verified every diagnostic array:',path)
