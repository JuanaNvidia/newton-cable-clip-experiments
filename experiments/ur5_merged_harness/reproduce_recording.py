"""Repeat the recorded phase-boundary restarts in a fresh experiment copy."""
from pathlib import Path
import subprocess,sys
root=Path(__file__).resolve().parent
steps=[['simulate.py','--iterations','60','--stop-after','2'],['simulate.py','--resume','motion_boundary.npz','--iterations','60','--stop-after','3'],['simulate.py','--resume','motion_boundary.npz','--iterations','60','--stop-after','4'],['controller_low_recovery.py','--resume','motion_boundary.npz','--iterations','80','--stop-after','5'],['controller_raised_recovery.py','--resume','motion_boundary.npz','--iterations','80','--stop-after','6'],['controller_raised_recovery.py','--resume','motion_boundary.npz','--iterations','80','--max-repairs-per-clip','3','--stop-after','8'],['controller_adaptive_recovery.py','--resume','motion_boundary.npz','--iterations','80','--max-repairs-per-clip','3','--stop-after','12'],['simulate.py','--resume','motion_boundary.npz','--iterations','80','--max-repairs-per-clip','3']]
for step in steps:
 print('+',sys.executable,*step,flush=True);subprocess.run([sys.executable,*step],cwd=root,check=True)
