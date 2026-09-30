"""Add assembly views around the half-speed Newton OpenGL insertion footage."""
from pathlib import Path
import subprocess,tempfile
ROOT=Path(__file__).resolve().parent
work=ROOT/'work'/'video';work.mkdir(parents=True,exist_ok=True)
def encode(source,dest,still=False):
    cmd=['ffmpeg','-y','-v','error']
    if still:cmd+=['-loop','1']
    cmd+=['-i',str(source)]
    if still:cmd+=['-t','2','-vf','fps=30,format=yuv420p']
    else:cmd+=['-vf','setpts=2*PTS','-r','30']
    cmd+=['-c:v','libx264','-crf','18','-pix_fmt','yuv420p',str(dest)]
    subprocess.run(cmd,check=True)
with tempfile.TemporaryDirectory(dir=work) as tmp:
    tmp=Path(tmp)
    for name,source,still in [('intro','assembly_before.png',True),('insertion','actual_assets_newton.mp4',False),('outro','actual_assembly.png',True)]:
        encode(ROOT/source,tmp/(name+'.mp4'),still)
    (tmp/'concat.txt').write_text("file 'intro.mp4'\nfile 'insertion.mp4'\nfile 'outro.mp4'\n")
    subprocess.run(['ffmpeg','-y','-v','error','-f','concat','-safe','0','-i',str(tmp/'concat.txt'),'-c','copy','-movflags','+faststart',str(ROOT/'six_cables_newton.mp4')],check=True)
