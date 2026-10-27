"""Evaluate swing-up honestly, preserving failures and wrapped-angle metrics."""
import csv
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
EXE = (ROOT / (sys.argv[1] if len(sys.argv)>1 else 'build/svd-check/double_pendulum.exe')).resolve()
OUT = ROOT/'logs/swingup_closed_loop'
OUT.mkdir(parents=True,exist_ok=True)
env = os.environ.copy()
compiler=shutil.which('g++')
env['PATH']=os.pathsep.join([str(ROOT/'build/deps/mujoco'),str(Path(compiler).parent) if compiler else '',env.get('PATH','')])
wrap=lambda a: math.remainder(a,2*math.pi)
results=[]
for q1,q2 in ((180,0),(-180,0),(90,0),(-90,0),(180,90),(180,-90),(90,90),(-90,-90)):
    output=OUT/f'q1_{q1}_q2_{q2}.csv'
    run=subprocess.run([str(EXE),'--swingup','--headless','--duration','5',
        '--q1',str(math.radians(q1)),'--q2',str(math.radians(q2)),
        '--output',str(output)],cwd=ROOT,env=env,capture_output=True,timeout=90)
    with output.open(encoding='utf-8',newline='') as stream:
        rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(stream)]
    complete=run.returncode==0 and abs(rows[-1]['time']-5)<0.003
    finite=all(math.isfinite(v) for r in rows for v in r.values())
    tail=[r for r in rows if r['time']>=4]
    angles=max((max(abs(wrap(r['q1'])),abs(wrap(r['q1']+r['q2']))) for r in tail),default=None)
    speeds=max((max(abs(r['dq1']),abs(r['dq1']+r['dq2'])) for r in tail),default=None)
    # Persistent settling, rather than a single crossing of a broad capture box.
    settled=bool(complete and finite and angles is not None and angles<0.02 and speeds<0.05 and
        max(abs(r['p']) for r in tail)<0.05 and max(abs(r['dp']) for r in tail)<0.05)
    result=dict(q1_deg=q1,q2_deg=q2,complete=complete,finite=finite,settled=settled,
        final_time=rows[-1]['time'],tail_angle=angles,tail_speed=speeds,
        peak_position=max(abs(r['p']) for r in rows),peak_force=max(abs(r['applied_force']) for r in rows),
        saturated_steps=sum(abs(r['requested_force']-r['applied_force'])>1e-9 for r in rows),
        failure=run.stderr.decode('utf-8',errors='replace').strip())
    results.append(result)
    print(json.dumps(result,ensure_ascii=True),flush=True)
report=dict(duration_s=5,successes=sum(r['settled'] for r in results),total=len(results),cases=results)
(OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
print(f"Swing-up success: {report['successes']}/{report['total']}; report: {OUT/'report.json'}")
