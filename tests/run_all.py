"""Run the project test suite from the repository root.

Use ``python tests/run_all.py --quick`` to skip the labelled acceptance test.
"""
from pathlib import Path
import argparse
import os
import shutil
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--quick", action="store_true", help="skip the acceptance-labelled test")
args = parser.parse_args()
env = os.environ.copy()
runtime = [str(root / "build" / "deps" / "mujoco")]
compiler = shutil.which("g++")
if compiler:
    runtime.append(str(Path(compiler).parent))
env["PATH"] = os.pathsep.join(runtime + [env.get("PATH", "")])
cmake = root / "build" / "deps" / "cmake" / "data" / "bin" / "cmake.exe"
if not cmake.exists():
    cmake = "cmake"
subprocess.run([str(cmake), "-S", str(root), "-B", str(root/"build/svd-check"), "-DCMAKE_BUILD_TYPE=Release"], cwd=root, env=env, check=True)
subprocess.run([str(cmake), "--build", str(root/"build/svd-check"), "-j", "2"], cwd=root, env=env, check=True)
ctest = cmake.parent / "ctest.exe" if isinstance(cmake, Path) else "ctest"
command = [str(ctest), "--test-dir", str(root/"build/svd-check"), "--output-on-failure"]
if args.quick:
    command += ["-LE", "acceptance"]
raise SystemExit(subprocess.run(command, cwd=root, env=env).returncode)
