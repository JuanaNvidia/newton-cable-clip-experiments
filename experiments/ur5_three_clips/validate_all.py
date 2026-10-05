"""Run every audit, preserving all reports even when one check fails."""
from pathlib import Path
import subprocess,sys,json
root=Path(__file__).resolve().parent
checks=['validate.py','check_clip_contacts.py','check_gripper_contacts.py','check_connector_contacts.py','check_robot_clearance.py'];codes={}
for script in checks:codes[script]=subprocess.run([sys.executable,str(root/script)],cwd=root).returncode
(root/'validation_summary.json').write_text(json.dumps({'passed':not any(codes.values()),'exit_codes':codes},indent=2)+'\n')
raise SystemExit(1 if any(codes.values()) else 0)
