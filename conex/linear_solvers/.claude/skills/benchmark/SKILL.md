---
name: benchmark
description: Run benchmark_solver against all SDPA instances and save results by git commit hash
user-invocable: true
---

# Benchmark: Run all SDPA instances

Run this benchmark to test the current solver against all downloaded SDPA instances.
Results are saved to a folder named after the current git commit hash.

## Steps

1. First verify git status is clean (no uncommitted changes). If dirty, STOP and tell the user to commit first.

2. Build the benchmark_solver:
```bash
cd /agent-workspace/conex/conex/linear_solvers/build && cmake --build . --target benchmark_solver
```

3. Get the current git commit hash:
```bash
cd /agent-workspace/conex/conex/linear_solvers && git rev-parse --short HEAD
```

4. Create the output directory:
```bash
mkdir -p /agent-workspace/conex/conex/linear_solvers/benchmark_results/<COMMIT_HASH>
```

5. Run benchmark_solver on each instance in `/agent-workspace/conex/conex/linear_solvers/benchmark_data/` and save output:
```bash
for f in /agent-workspace/conex/conex/linear_solvers/benchmark_data/*.dat-s; do
  name=$(basename "$f" .dat-s)
  echo "Running $name..."
  /agent-workspace/conex/conex/linear_solvers/build/benchmark_solver "$f" \
    > /agent-workspace/conex/conex/linear_solvers/benchmark_results/<COMMIT_HASH>/${name}.txt 2>&1
done
```

6. Display a summary table showing instance name, factorizations, solves, mu, and time for each instance. Parse this from the ThetaCont line in each output file.

7. Report the output directory path so the user can inspect individual results.
