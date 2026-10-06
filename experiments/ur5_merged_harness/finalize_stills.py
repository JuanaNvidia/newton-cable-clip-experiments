"""Provide clip stills when conditional recovery replaces a scheduled bulk grasp."""
from pathlib import Path
import numpy as np,shutil
from motion_io import load_motion
root=Path(__file__).resolve().parent;d=load_motion(root/'motion.npz')
for k in range(3):
 path=root/f'clip{k+1}.png'
 if path.exists():continue
 indices=np.flatnonzero(d['phases']==f'repair_{k+1}')
 if len(indices):
  # Last frame of the final attempt, rounded to the 15 fps renderer sample.
  frame=int(indices[-1])//4;source=root/'work/frames_ur5_cable_clip'/f'frame_{frame:04}.png';shutil.copy2(source,path)
 else:shutil.copy2(root/'assembly_after.png',path)
