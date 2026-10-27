import csv
import math
import os
import subprocess
from pathlib import Path

root = Path(__file__).resolve().parents[2]
exe = root / "build" / "svd-check" / "double_pendulum.exe"
env = os.environ.copy()
env["PATH"] = str(root / "build" / "deps" / "mujoco") + ";D:/mingw/MinGW/bin;" + env.get("PATH", "")
cases = [
    ("0.5deg", 0.00872665, 0.0, 0.0, 0.0),
    ("1deg", 0.0174533, 0.0, 0.0, 0.0),
    ("2deg", 0.0349066, 0.0, 0.0, 0.0),
    ("velocity", 0.0, 0.0, 0.2, 0.0),
]
for name, q1, q2, dq1, dq2 in cases:
    command = [str(exe), "--lmpc", "--headless", "--duration", "2",
               "--q1", str(q1), "--q2", str(q2),
               "--dq1", str(dq1), "--dq2", str(dq2),
               "--output", str(root / "logs" / ("lmpc_" + name + ".csv"))]
    completed = subprocess.run(command, env=env, capture_output=True, text=True, encoding="utf-8", errors="replace")
    if completed.returncode != 0:
        raise RuntimeError(name + ": " + completed.stderr[-500:])
    path = root / "logs" / ("lmpc_" + name + ".csv")
    with path.open(newline="", encoding="utf-8") as stream:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    assert rows and abs(rows[-1]["time"] - 2.0) < 0.003
    assert all(math.isfinite(v) for row in rows for v in row.values())
    assert all(abs(row["applied_force"]) <= 20.0 + 1e-10 for row in rows)
    tail = max(abs(row[k]) for row in rows if row["time"] >= 1.5
               for k in ("p", "dp", "q1", "dq1", "q2", "dq2"))
    peak = max(abs(row["applied_force"]) for row in rows)
    saturated = sum(abs(row["requested_force"] - row["applied_force"]) > 1e-9 for row in rows)
    print(f"{name}: tail_max={tail:.6g}, peak_force={peak:.6g}, saturated={saturated}")
print("LMPC closed-loop checks passed")
