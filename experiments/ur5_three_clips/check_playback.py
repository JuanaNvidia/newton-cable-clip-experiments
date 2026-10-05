from pathlib import Path
import json,hashlib,subprocess,numpy as np
from pxr import Usd,UsdGeom
from motion_io import load_motion,motion_sha256
r=Path(__file__).resolve().parent;d=load_motion(r/'motion.npz');q=d['poses'];stage=Usd.Stage.Open(str(r/'ur5_playback.usdc'));assert stage.GetTimeCodesPerSecond()==60 and stage.GetEndTimeCode()==len(q)-1
samples=[(int(d['plug_bodies'][0]),'/World/Connector_1'),(int(d['plug_bodies'][1]),'/World/Connector_2'),(int(d['rod_ids'][0]),'/World/Cable_0/Segment_000'),(int(d['clip_top_bodies'][0]),'/World/Clip_1/Top'),(int(d['clip_top_bodies'][2]),'/World/Clip_3/Top'),(int(d['robot_start']),'/World/Robot/'+str(d['robot_names'][0]))]
max_error=0.
for body,path in samples:
 ops=UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()
 for frame in [0,(len(q)//3)//4*4,(2*len(q)//3)//4*4,len(q)-1]:
  pos=np.array(ops[0].Get(frame));rot=ops[1].Get(frame);quat=np.r_[list(rot.GetImaginary()),rot.GetReal()];max_error=max(max_error,float(np.max(abs(pos-q[frame,body,:3]))),float(min(np.linalg.norm(quat-q[frame,body,3:]),np.linalg.norm(quat+q[frame,body,3:]))))
assert max_error<1e-6
assert not any('Physics' in str(schema) for prim in stage.Traverse() for schema in prim.GetAppliedSchemas())
probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration,size','-show_entries','stream=codec_name,width,height,r_frame_rate','-of','json',str(r/'ur5_cable_clip.mp4')]))
assert probe['streams'][0]['codec_name']=='h264' and probe['streams'][0]['width']==1280 and probe['streams'][0]['height']==960
report=dict(kind='Visual playback integrity, separate from physics validation',passed=True,motion_sha256=motion_sha256(r/'motion.npz'),sampled_transform_max_error=max_error,usd_time_codes_per_second=60,usd_pose_samples_per_second=15,usd_end_frame=int(stage.GetEndTimeCode()),no_active_physics_schemas=True,video=probe)
(r/'playback_validation.json').write_text(json.dumps(report,indent=2)+'\n');print('Playback integrity passed; video and USD match expected format and sampled solved poses.')
