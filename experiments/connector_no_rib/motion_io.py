"""Load a local recording or lossless frame chunks used for GitHub distribution."""
from pathlib import Path
import json
import numpy as np

def load_motion(path):
 path=Path(path)
 if path.exists():return np.load(path)
 manifest=path.with_suffix('.manifest.json')
 spec=json.loads(manifest.read_text());chunks=[np.load(path.parent/x) for x in spec['chunks']]
 result={}
 for key in chunks[0].files:
  result[key]=np.concatenate([c[key] for c in chunks],axis=0) if key in spec['frame_arrays'] else chunks[0][key]
 return result

def motion_sha256(path):
 import hashlib
 path=Path(path)
 if path.exists():return hashlib.sha256(path.read_bytes()).hexdigest()
 spec=json.loads(path.with_suffix('.manifest.json').read_text())
 for name in spec['chunks']:
  assert hashlib.sha256((path.parent/name).read_bytes()).hexdigest()==spec['sha256'][name],f'Changed motion chunk: {name}'
 return spec['original_npz_sha256']
