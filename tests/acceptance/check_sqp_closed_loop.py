"""Batch smoke test for the SQP baseline controller."""
import csv
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
EXE = (ROOT / (sys.argv[1] if len(sys.argv) > 1 else "build/svd-check/double_pendulum.exe")).resolve()
OUT = ROOT / "logs/sqp_closed_loop"
OUT.mkdir(parents=True, exist_ok=True)
env = os.environ.copy()
env["PATH"] = os.pathsep.join([str(ROOT / "build/deps/mujoco"), r"D:\mingw\MinGW\bin", env.get("PATH", "")])

for q1_deg, q2_deg in ((0.5, 0.0), (1.0, 0.0), (1.0, 1.0), (2.0, 0.0)):
    output = OUT / f"q1_{q1_deg:g}_q2_{q2_deg:g}.csv"
    started = time.perf_counter()
    run = subprocess.run([
        str(EXE), "--sqp", "--headless", "--duration", "1.0",
        "--q1", str(q1_deg * 3.141592653589793 / 180.0),
        "--q2", str(q2_deg * 3.141592653589793 / 180.0),
        "--output", str(output)], cwd=ROOT, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    elapsed = time.perf_counter() - started
    if run.returncode != 0:
        raise RuntimeError(run.stderr.decode("utf-8", errors="replace"))
    with output.open(newline="", encoding="utf-8") as stream:
        rows = [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]
    assert rows and abs(rows[-1]["time"] - 1.0) < 0.003
    tail = [row for row in rows if row["time"] >= 0.8]
    tail_error = max(max(abs(row[key]) for row in tail)
                     for key in ("p", "dp", "q1", "dq1", "q2", "dq2"))
    peak_force = max(abs(row["applied_force"]) for row in rows)
    print(f"q1={q1_deg:g}°, q2={q2_deg:g}°: tail_error={tail_error:.4g}, "
          f"peak_force={peak_force:.4g} N, wall_time={elapsed:.3f} s")
print("SQP closed-loop smoke checks passed")
