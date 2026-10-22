"""Run from the project root after building; overwrites logs/lqr_check.csv."""
import csv
import math
from pathlib import Path
import subprocess
import sys

exe = Path(sys.argv[1] if len(sys.argv) > 1 else "build/double_pendulum.exe").resolve()
log = Path("logs/lqr_check.csv")
for q1, q2 in [(0.02, 0), (0, 0.02), (0.02, -0.02), (-0.02, -0.02)]:
    run = subprocess.run([str(exe), "--lqr", "--headless", "--duration", "10",
                          "--q1", str(q1), "--q2", str(q2), "--output", str(log)],
                         capture_output=True)
    if run.returncode:
        raise RuntimeError(run.stdout.decode("utf-8", errors="replace") +
                           run.stderr.decode("utf-8", errors="replace"))
    with log.open(newline="", encoding="utf-8") as stream:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    assert len(rows) > 1
    assert all(math.isfinite(v) for row in rows for v in row.values())
    assert abs(rows[-1]["time"] - 10) < 0.003
    states = ["p", "dp", "q1", "dq1", "q2", "dq2"]
    final_error = max(abs(rows[-1][k]) for k in states)
    # Settling must persist over the final second, not just at one zero crossing.
    tail_error = max(abs(row[k]) for row in rows if row["time"] >= 9 for k in states)
    assert tail_error < 1e-3, (q1, q2, tail_error)
    assert all(abs(row["applied_force"]) <= 20 + 1e-10 for row in rows)
    saturated = sum(abs(row["requested_force"] - row["applied_force"]) > 1e-9
                    for row in rows[1:])
    peak_force = max(abs(row["applied_force"]) for row in rows)
    peak_position = max(abs(row["p"]) for row in rows)
    print(f"q1={q1}, q2={q2}: final_max={final_error:.3g}, "
          f"last_second_max={tail_error:.3g}, peak_force={peak_force:.4g} N, "
          f"peak_position={peak_position:.4g} m, saturated_steps={saturated}")

# Preserve CLI mode semantics: conflicting force requests must fail.
conflict = subprocess.run([str(exe), "--lqr", "--force", "1"], capture_output=True)
assert conflict.returncode != 0
print("LQR nonlinear small-angle closed-loop checks passed")
