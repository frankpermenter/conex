---
name: cleanup
description: Run tests, benchmarks, and find unused code in the linear_solvers directory
user-invocable: true
---

# Cleanup: Test Coverage and Dead Code Report

Run all tests and benchmarks, then search for unused classes, functions, and dead code.

## Steps

### 1. Build and run all tests

```bash
cd /agent-workspace/conex/linear_solvers
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="" . 2>&1 | tail -2
make -j$(nproc) 2>&1 | tail -5
ctest --output-on-failure 2>&1
```

Report: number of tests, pass/fail, any failures.

### 2. Run benchmarks on available MTX files

```bash
./profile_mtx --randomize \
  /agent-workspace/interfaces/python/test/benchmark_data/ash958/ash958.mtx \
  /agent-workspace/interfaces/python/test/benchmark_data/illc1850/illc1850.mtx \
  /agent-workspace/interfaces/python/test/benchmark_data/well1850/well1850.mtx \
  2>&1
```

Report: timing and residuals for each matrix.

### 3. Find unused code

Search for classes, functions, and methods that are declared/defined but never referenced outside their own file. Focus on:

- **Unused classes**: `class Foo` where `Foo` never appears in other files
- **Unused functions**: function definitions where the function name never appears in other .cc/.h files
- **Dead methods**: virtual methods with no overrides and no calls
- **Unreachable code**: `#if 0` blocks, commented-out code

Search scope: `conex/linear_solvers/conex/` (common/, tree_solver/, test/)

Use `grep -r` to find declarations and cross-reference with usages.

### 4. Produce report

Output a summary with sections:
- **Test Results**: pass/fail counts
- **Benchmark Results**: table of timing/residuals
- **Unused Code**: list of candidates for removal with file:line references
- **Recommendations**: what to remove, what to investigate further
