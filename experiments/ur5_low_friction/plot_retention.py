from pathlib import Path
import sys,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parent;sys.path.insert(0,str(root))
from motion_io import load_motion
from audit_geometry import retained_at_clips
q=load_motion(root/'motion.npz');poses=q['poses'];indices=np.arange(0,len(poses),15);times=indices/60
counts=np.array([np.sum(retained_at_clips(poses[i],q['rod_ids'],float(q['segment_length']),q['clip_top_bodies']),axis=1) for i in indices]);meta=json.loads((root/'motion.json').read_text())
fig,axes=plt.subplots(3,1,figsize=(12,7),sharex=True)
for k,ax in enumerate(axes):
 ax.step(times,counts[:,k],where='post',color=['#bb3542','#2678a9','#399370'][k]);ax.set(ylim=(-.3,6.4),yticks=range(7),ylabel=f'Clip {k+1}\nWires in channel');ax.grid(alpha=.2)
 for e in meta['events']:
  if e['event']=='opposite_side_wire_selection' and e['clip']==k+1:ax.axvline(e['time'],color='#999999',ls='--',lw=.8);ax.text(e['time']+.3,.4,f"wire {e['cable']}\nside {e['side']:+}",fontsize=7)
axes[-1].set_xlabel('Simulation time (seconds)');fig.suptitle('Wire counts inside each channel and below its moving lid');fig.tight_layout();fig.savefig(root/'retention_timeline.png',dpi=160)
(root/'retention_timeline.json').write_text(json.dumps(dict(seconds=times.tolist(),counts=counts.tolist(),sampling_hz=4),indent=2)+'\n')
