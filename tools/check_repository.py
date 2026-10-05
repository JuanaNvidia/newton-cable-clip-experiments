"""CPU-only integrity checks for the checked-in experiment results."""
from pathlib import Path
import argparse,hashlib,json
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--write-manifest',action='store_true');args=p.parse_args()
exts={'.npz','.mp4','.png','.stl','.usd','.usda','.usdc','.json','.dae','.urdf','.xml','.yaml'}
files=sorted(p for top in ['experiments','archive'] for p in (ROOT/top).rglob('*') if p.is_file() and p.suffix in exts and 'work' not in p.relative_to(ROOT).parts)
manifest={str(p.relative_to(ROOT)):{'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in files}
path=ROOT/'artifact_manifest.json'
if args.write_manifest:path.write_text(json.dumps(manifest,indent=2)+'\n')
expected=json.loads(path.read_text())
assert manifest==expected,'Artifact set or content changed. Review before updating the manifest.'
for top in ['experiments','archive','tools']:
 for source in (ROOT/top).rglob('*.py'):compile(source.read_text(),str(source),'exec')
for folder in ['six_cable_newton','long_cable_newton','three_inch_arch','hinged_connector_insertion','fixed_receiver_insertion','connector_and_clip']:
 report=json.loads((ROOT/'experiments'/folder/'validation.json').read_text());assert report['passed'],folder
assert json.loads((ROOT/'experiments/long_cable_newton/arch_validation.json').read_text())['passed']
robot=ROOT/'experiments/ur5_cable_clip';report=json.loads((robot/'validation.json').read_text())
assert report['passed']==all(report['checks'].values()),'Robot report must preserve failed diagnostics'
assert report['motion_sha256']==hashlib.sha256((robot/'motion.npz').read_bytes()).hexdigest()
assert report['checks']['finite'] and report['checks']['connector_stays_seated']
interior=ROOT/'experiments/ur5_interior_clip';r=json.loads((interior/'validation.json').read_text())
assert r['passed']==all(r['checks'].values())
assert r['motion_sha256']==hashlib.sha256((interior/'motion.npz').read_bytes()).hexdigest()
assert r['checks']['finite'] and r['checks']['connector_stays_seated'] and r['checks']['clip_at_least_100mm_inside_table']
with np.load(interior/'motion.npz') as d:
 assert np.isfinite(d['poses']).all() and d['distal_anchors'].shape==(6,3)
 assert int(d['distal_body'])<int(d['robot_start'])
opposite=ROOT/'experiments/ur5_opposite_pull';r=json.loads((opposite/'validation.json').read_text());sha=hashlib.sha256((opposite/'motion.npz').read_bytes()).hexdigest()
assert r['passed']==all(r['checks'].values()) and r['motion_sha256']==sha
for name in ['contact_validation','gripper_contact_validation','robot_clearance_validation']:
 assert json.loads((opposite/(name+'.json')).read_text())['motion_sha256']==sha
assert r['checks']['finite'] and r['checks']['connector_stays_seated']
assert r['checks']['recovery_trigger_matches_retention_check'] and r['checks']['feedback_matches_recorded_geometry']
with np.load(opposite/'motion.npz') as d:
 assert np.isfinite(d['poses']).all() and d['recovery_retention'].shape==(len(d['poses']),6)
for folder,name in [('six_cable_newton','actual_motion'),('long_cable_newton','actual_motion'),('long_cable_newton','arch_motion'),('three_inch_arch','arch_motion'),('ur5_cable_clip','motion'),('connector_and_clip','motion'),('connector_and_clip','original_spring_motion'),('fixed_receiver_insertion','motion'),('hinged_connector_insertion','motion'),('hinged_connector_insertion','no_receiver_contact')]:
 with np.load(ROOT/'experiments'/folder/(name+'.npz'),allow_pickle=False) as d:
  assert np.isfinite(d['poses']).all(),(folder,name)
  assert d['poses'].shape[-1]==7 and int(d['fps'])==60
for name in ['clipTop.stl','clipBottom.stl']:
 assert (ROOT/'experiments/six_cable_newton'/name).read_bytes()==(opposite/name).read_bytes()
 assert (ROOT/'experiments/six_cable_newton'/name).read_bytes()==(interior/name).read_bytes()
 assert (ROOT/'experiments/six_cable_newton'/name).read_bytes()==(ROOT/'experiments/long_cable_newton'/name).read_bytes()
 assert (ROOT/'experiments/six_cable_newton'/name).read_bytes()==(ROOT/'experiments/connector_and_clip'/name).read_bytes()
 assert (ROOT/'experiments/six_cable_newton'/name).read_bytes()==(ROOT/'experiments/ur5_cable_clip'/name).read_bytes()
assert not json.loads((ROOT/'archive/coarse_960hz/validation.json').read_text())['passed'],'Historical failed control must stay labeled failed'
assert not json.loads((ROOT/'experiments/connector_and_clip/original_spring_motion_validation.json').read_text())['passed'],'Original spring comparison must retain its failed closed-clip outcome'
three=ROOT/'experiments/ur5_three_clips'
import importlib.util
spec=importlib.util.spec_from_file_location('three_motion_io',three/'motion_io.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
d=module.load_motion(three/'motion.npz');sha=module.motion_sha256(three/'motion.npz')
assert np.isfinite(d['poses']).all() and int(d['fps'])==60
assert d['latch_history'][0].sum()==0 and d['latch_history'][-1].sum()==2
assert len(d['clip_top_bodies'])==3 and len(d['plug_bodies'])==2
report=json.loads((three/'validation.json').read_text());assert report['motion_sha256']==sha and report['passed']==all(report['checks'].values())
for name in ['contact_validation','gripper_contact_validation','robot_clearance_validation','connector_contact_validation']:
 assert json.loads((three/(name+'.json')).read_text())['motion_sha256']==sha
playback=json.loads((three/'playback_validation.json').read_text());assert playback['passed'] and playback['motion_sha256']==sha
provenance=json.loads((three/'recording_provenance.json').read_text());assert provenance['motion_sha256']==sha and provenance['prefix_pose_arrays_verified_identical']
assert provenance['recorded_frames']==len(d['poses'])
for name in ['clipTop.stl','clipBottom.stl']:assert (three/name).read_bytes()==(opposite/name).read_bytes()
assert all(p.stat().st_size<100_000_000 for p in three.rglob('*') if p.is_file())
for diagnostic_name in ['lower_entry','angled_bundle']:
 diag=json.loads((three/f'diagnostics/{diagnostic_name}_tail.json').read_text());tail=np.load(three/f'diagnostics/{diagnostic_name}_tail.npz');n=diag['prefix_frames'];assert n<=len(d['poses'])
 for key in tail.files:
  if key in diag['frame_arrays']:
   prefix=d[key][:n];assert hashlib.sha256(prefix.tobytes()).hexdigest()==diag['prefix_array_sha256'][key];value=np.concatenate([prefix,tail[key]])
  else:value=tail[key]
  expected=diag['arrays'][key];assert list(value.shape)==expected['shape'] and str(value.dtype)==expected['dtype'];assert hashlib.sha256(value.tobytes()).hexdigest()==expected['sha256'],key
 assert json.loads((three/f'diagnostics/{diagnostic_name}_validation.json').read_text())['motion_sha256']==diag['original_npz_sha256']

no_rib=ROOT/'experiments/connector_no_rib'
selection=json.loads((no_rib/'selection.json').read_text());name=selection['selected']
r=json.loads((no_rib/(name+'_validation.json')).read_text());sha=hashlib.sha256((no_rib/(name+'.npz')).read_bytes()).hexdigest()
assert r['motion_sha256']==sha and r['passed']==all(r['checks'].values()) and r['passed']
for report in ['connector_contact_validation','robot_clearance_validation']:
 assert json.loads((no_rib/(name+'_'+report+'.json')).read_text())['motion_sha256']==sha
with np.load(no_rib/(name+'.npz')) as m:assert np.isfinite(m['poses']).all() and len(m['poses'])==1200
parts=json.loads((no_rib/'assets/geometry.json').read_text())['plug']
assert not any('handling_rib' in part['name'] for part in parts)
assert all(part['collision'] for part in parts if part['name'].startswith(('grip_ear','top_ridge','rear_ridge','blue_wire_support')))
assert not json.loads((no_rib/(name+'.json')).read_text())['existing_fingertip_support_lips']

print(f'PASS: {len(files)} artifact hashes, finite reference motions, expected validation statuses, identical clip meshes, Python syntax.')
