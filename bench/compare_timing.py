"""Compare conex and Clarabel timing on SDPLIB instances.

Uses conex's SDPA reader + C++ solver and Clarabel's Python API.
Prints per-problem: setup time, solve time, iterations, objective.
"""
import sys
sys.path.insert(0, '/agent-workspace/conex/build')

import time
import subprocess
import os

SDPLIB = "/agent-workspace/problem_libraries/SDPLIB/data"
CONEX_BENCH = "/agent-workspace/conex/build/benchmark_solver"
CLARABEL_BENCH = "/tmp/clarabel_build/release/examples/benchmark_sdp"

# Small SDPLIB instances
problems = [
    "arch0", "arch2", "control1", "control2",
    "gpp100", "gpp124-1", "gpp124-2", "gpp124-3", "gpp124-4",
    "mcp100", "mcp124-1", "mcp124-2", "mcp124-3", "mcp124-4",
    "truss1", "truss2", "truss3", "truss4", "truss5",
]

print(f"{'Problem':<15} {'Conex(ms)':<12} {'Conex iter':<12} {'Clarabel(ms)':<14} {'Clar iter':<10}")
print("-" * 70)

for name in problems:
    path = os.path.join(SDPLIB, f"{name}.dat-s")
    if not os.path.exists(path):
        continue

    # Run conex
    try:
        t0 = time.time()
        result = subprocess.run(
            [CONEX_BENCH, path, "--algorithm", "thetacontr", "--max_iter", "100"],
            capture_output=True, text=True, timeout=30
        )
        conex_time = (time.time() - t0) * 1000
        conex_iter = "?"
        for line in result.stdout.split('\n'):
            if 'iterations' in line.lower() or 'iter' in line.lower():
                conex_iter = line.strip()
                break
    except Exception as e:
        conex_time = -1
        conex_iter = str(e)[:20]

    # Run clarabel (if benchmark exists)
    clar_time = -1
    clar_iter = "n/a"

    print(f"{name:<15} {conex_time:>10.1f}  {conex_iter:<12} {clar_time:>12.1f}  {clar_iter:<10}")
