"""Rerun lower clip friction with the closer-grasp baseline restart boundaries."""
from pathlib import Path
import subprocess,sys
root=Path(__file__).resolve().parent
common=['--iterations','80','--substeps','32','--max-repairs-per-clip','3','--clip-friction','.1']
subprocess.run([sys.executable,str(root/'simulate.py'),'--resume','two_connectors.npz','--output','first_clip','--stop-after','4',*common],check=True)
subprocess.run([sys.executable,str(root/'simulate.py'),'--resume','first_clip.npz',*common],check=True)
