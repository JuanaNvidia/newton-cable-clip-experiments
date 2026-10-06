"""Rerun the closer-grasp recording with the same phase-boundary restart."""
from pathlib import Path
import subprocess,sys
root=Path(__file__).resolve().parent
common=['--iterations','80','--substeps','32','--max-repairs-per-clip','3']
subprocess.run([sys.executable,str(root/'simulate.py'),'--resume','two_connectors.npz','--stop-after','4',*common],check=True)
subprocess.run([sys.executable,str(root/'simulate.py'),'--resume','motion_boundary.npz',*common],check=True)
