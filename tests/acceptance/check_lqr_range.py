"""Measure sampled LQR recovery ranges with the current nonlinear model.

Run from the project root: python tests/acceptance/check_lqr_range.py
Reports and per-case logs go to logs/lqr_range/.
This is a numerical experiment, not a proof of a region of attraction.
"""
import csv
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
EXE = (ROOT / (sys.argv[1] if len(sys.argv) > 1 else 'build/double_pendulum.exe')).resolve()
OUT = ROOT / 'logs/lqr_range'
OUT.mkdir(parents=True, exist_ok=True)
ENV = os.environ.copy()
compiler = shutil.which('g++')
ENV['PATH'] = os.pathsep.join([str(ROOT / 'build/deps/mujoco'), str(Path(compiler).parent) if compiler else '', ENV.get('PATH', '')])
DURATION = 15.0
STATES = ('p', 'dp', 'q1', 'dq1', 'q2', 'dq2')
# Explicit physical thresholds, checked throughout the final second.
TOLERANCES = dict(p=0.001, dp=0.001, q1=0.001, dq1=0.001, q2=0.001, dq2=0.001)
results = []
cache = {}

def run_case(q1_deg, q2_deg, dq1=0.0, dq2=0.0):
    key = (round(q1_deg, 8), round(q2_deg, 8), dq1, dq2)
    if key in cache:
        return cache[key]
    log = OUT / ('case_' + '_'.join(f'{v:g}' for v in key) + '.csv')
    proc = subprocess.run([str(EXE), '--lqr', '--headless', '--duration', str(DURATION),
        '--q1', str(math.radians(q1_deg)), '--q2', str(math.radians(q2_deg)),
        '--dq1', str(dq1), '--dq2', str(dq2), '--output', str(log)],
        cwd=ROOT, env=ENV, capture_output=True, timeout=30)
    if not log.exists():
        raise RuntimeError(proc.stdout.decode('utf-8', errors='replace') + proc.stderr.decode('utf-8', errors='replace'))
    with log.open(encoding='utf-8', newline='') as stream:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    if not rows:
        raise RuntimeError('Empty simulation log: ' + str(log))
    finite = all(math.isfinite(v) for row in rows for v in row.values())
    complete = proc.returncode == 0 and abs(rows[-1]['time'] - DURATION) < 0.003
    tail = [row for row in rows if row['time'] >= DURATION - 1]
    tail_max = {k: max((abs(row[k]) for row in tail), default=math.inf) for k in STATES}
    passed = finite and complete and all(tail_max[k] < TOLERANCES[k] for k in STATES)
    # A completed fall must not be mistaken for recovery simply because simulation continued.
    saturated = [r for r in rows[1:] if abs(r['requested_force'] - r['applied_force']) > 1e-9]
    result = dict(q1_deg=q1_deg, q2_deg=q2_deg, dq1=dq1, dq2=dq2, passed=passed,
        final_time=rows[-1]['time'], peak_force=max(abs(r['applied_force']) for r in rows),
        peak_requested_force=max(abs(r['requested_force']) for r in rows),
        peak_position=max(abs(r['p']) for r in rows), saturated_steps=len(saturated),
        tail_saturated_steps=sum(abs(r['requested_force']-r['applied_force']) > 1e-9 for r in tail),
        tail_max=tail_max, stderr=proc.stderr.decode('utf-8', errors='replace').strip())
    cache[key] = result
    results.append(result)
    print(f"q=({q1_deg:g},{q2_deg:g}) deg dq=({dq1:g},{dq2:g}): {'PASS' if passed else 'FAIL'} peak_p={result['peak_position']:.4g} saturated={len(saturated)} tail={max(tail_max.values()):.3g}", flush=True)
    return result

boundaries = []
for name, direction in [('q1_only',(1,0)), ('q2_only',(0,1)), ('same_joint_angles',(1,1)), ('opposite_joint_angles',(1,-1))]:
    for sign in (1, -1):
        last_pass = 0.0
        first_fail = None
        for i in range(1, 61):
            angle = i * 0.5
            r = run_case(sign*angle*direction[0], sign*angle*direction[1])
            if not r['passed']:
                first_fail = angle
                break
            last_pass = angle
        if first_fail is not None:
            for i in range(1, 5):
                angle = round(last_pass + 0.1, 8)
                if angle >= first_fail - 1e-8:
                    break
                r = run_case(sign*angle*direction[0], sign*angle*direction[1])
                if not r['passed']:
                    first_fail = angle
                    break
                last_pass = angle
        boundaries.append(dict(direction=name, sign=sign, last_pass_deg=last_pass, first_fail_deg=first_fail))

# Representative independent angle combinations in a small square.
for q1 in (-2, -1, 0, 1, 2):
    for q2 in (-2, -1, 0, 1, 2):
        run_case(q1, q2)
for dq1, dq2 in ((0.2,0), (-0.2,0), (0,0.2), (0,-0.2)):
    run_case(0, 0, dq1, dq2)
for result in results:
    result['tail_max'] = {k: (v if math.isfinite(v) else None) for k, v in result['tail_max'].items()}
report = dict(duration_s=DURATION, tolerances=TOLERANCES, boundaries=boundaries, cases=results)
(OUT / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
with (OUT / 'summary.csv').open('w', encoding='utf-8-sig', newline='') as stream:
    fields = ['q1_deg','q2_deg','dq1','dq2','passed','final_time','peak_force','peak_requested_force','peak_position','saturated_steps','tail_saturated_steps']
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()
    writer.writerows({k:r[k] for k in fields} for r in results)
print('BOUNDARIES ' + json.dumps(boundaries), flush=True)
print(f'{len(results)} cases; reports: {OUT}', flush=True)
