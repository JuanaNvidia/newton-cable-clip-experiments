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
for folder,name in [('six_cable_newton','actual_motion'),('long_cable_newton','actual_motion'),('long_cable_newton','arch_motion'),('three_inch_arch','arch_motion'),('ur5_cable_clip','motion'),('connector_and_clip','motion'),('connector_and_clip','original_spring_motion'),('fixed_receiver_insertion','motion'),('hinged_connector_insertion','motion'),('hinged_connector_insertion','no_receiver_contact')]:
 with np.load(ROOT/'experiments'/folder/(name+'.npz'),allow_pickle=False) as d:
  assert np.isfinite(d['poses']).all(),(folder,name)
  assert d['poses'].shape[-1]==7 and int(d['fps'])==60
for name in ['clipTop.stl','clipBottom.stl']:
 assert (ROOT/'experiments/six_cable_newton'/name).read_bytes()==(interior/name).read_bytes()
 assert (ROOT/'experiments/six_cable_newton'/name).read_bytes()==(ROOT/'experiments/long_cable_newton'/name).read_bytes()
 assert (ROOT/'experiments/six_cable_newton'/name).read_bytes()==(ROOT/'experiments/connector_and_clip'/name).read_bytes()
 assert (ROOT/'experiments/six_cable_newton'/name).read_bytes()==(ROOT/'experiments/ur5_cable_clip'/name).read_bytes()
assert not json.loads((ROOT/'archive/coarse_960hz/validation.json').read_text())['passed'],'Historical failed control must stay labeled failed'
assert not json.loads((ROOT/'experiments/connector_and_clip/original_spring_motion_validation.json').read_text())['passed'],'Original spring comparison must retain its failed closed-clip outcome'
print(f'PASS: {len(files)} artifact hashes, finite reference motions, expected validation statuses, identical clip meshes, Python syntax.')
