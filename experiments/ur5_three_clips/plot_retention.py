"""Plot measured seating/retention from the recorded Newton poses."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from motion_io import load_motion,motion_sha256
from audit_geometry import retained_at_clips
root=Path(__file__).resolve().parent;d=load_motion(root/'motion.npz');fps=int(d['fps']);poses=d['poses'];rod_ids=d['rod_ids'];ds=float(d['segment_length']);tops=d['clip_top_bodies'];latch_history=d['latch_history'];frames=sorted(set(range(0,len(poses),15))|{len(poses)-1});t=np.array(frames)/fps
masks=np.array([retained_at_clips(poses[f],rod_ids,ds,tops) for f in frames]);counts=masks.sum(axis=2);latches=latch_history[frames].sum(axis=1)
report=dict(motion_sha256=motion_sha256(root/'motion.npz'),sample_hz=4,times_seconds=t.tolist(),retained_counts=counts.tolist(),latched_connector_counts=latches.tolist(),final_masks=masks[-1].tolist(),method='Original geometry retention criterion evaluated on recorded poses; task occupancy, not a force or contact-quality validation.')
(root/'retention_timeline.json').write_text(json.dumps(report,indent=2)+'\n')
fig,axes=plt.subplots(4,1,figsize=(12,7),sharex=True,layout='constrained');fig.suptitle('Recorded assembly: connector latches and cable retention',fontsize=16)
axes[0].step(t,latches,where='post',color='#334155',lw=1.7);axes[0].set_ylabel('Locked\nsockets');axes[0].set_yticks([0,1,2]);axes[0].set_ylim(-.15,2.3)
for k,ax in enumerate(axes[1:]):
 ax.step(t,counts[:,k],where='post',color=['#0284c7','#059669','#d97706'][k],lw=1.6);ax.set_ylabel(f'Clip {k+1}\ncables');ax.set_yticks([0,2,4,6]);ax.set_ylim(-.4,6.7);ax.axhline(6,color='#9ca3af',lw=.7,ls=':')
for ax in axes:ax.grid(axis='y',alpha=.2);ax.spines[['top','right']].set_visible(False)
axes[-1].set_xlabel('Recorded time (s)');axes[-1].set_xlim(0,t[-1]);fig.savefig(root/'retention_timeline.png',dpi=160);plt.close(fig);print('Saved measured retention timeline.')
