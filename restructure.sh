#!/bin/bash
# Restructure conex repo: move linear_solvers to top level, legacy code to legacy/
set -euo pipefail
cd "$(dirname "$0")"

# Create target directories
mkdir -p src/conex tests bench/data doc legacy/test legacy/interfaces legacy/examples legacy/third_party legacy/tools scripts python

# 1. Move active source code (preserving conex/ prefix for includes)
git mv conex/linear_solvers/conex/common src/conex/common
git mv conex/linear_solvers/conex/algorithms src/conex/algorithms
git mv conex/linear_solvers/conex/tree_solver src/conex/tree_solver
git mv conex/linear_solvers/conex/gpu_tree_solver src/conex/gpu_tree_solver

# 2. Move tests
for f in conex/linear_solvers/conex/test/*_test.cc; do
  [ -f "$f" ] && git mv "$f" tests/
done
git mv conex/linear_solvers/conex/test/kkt_residuals.h tests/

# 3. Move benchmarks
for f in conex/linear_solvers/conex/test/benchmark.cc \
         conex/linear_solvers/conex/test/benchmark_center.cc \
         conex/linear_solvers/conex/test/benchmark_qp.cc \
         conex/linear_solvers/conex/test/benchmark_solver.cc \
         conex/linear_solvers/conex/test/analyze_tree.cc \
         conex/linear_solvers/conex/test/canary.cc \
         conex/linear_solvers/conex/test/check_ct.cc \
         conex/linear_solvers/conex/test/compare_embedding.cc \
         conex/linear_solvers/conex/test/debug_problem.cc \
         conex/linear_solvers/conex/test/eval_embedding.cc \
         conex/linear_solvers/conex/test/eval_thetacontr.cc \
         conex/linear_solvers/conex/test/gpu_benchmark_cudss.cc \
         conex/linear_solvers/conex/test/hybrid_coldstart.cc \
         conex/linear_solvers/conex/test/lqr_benchmark.cc \
         conex/linear_solvers/conex/test/multi_rhs_benchmark.cc \
         conex/linear_solvers/conex/test/plot_eval_thetacontr.py \
         conex/linear_solvers/conex/test/profile_mtx.cc \
         conex/linear_solvers/conex/test/sign_check.cc \
         conex/linear_solvers/conex/test/solver_comparison.cc \
         conex/linear_solvers/conex/test/test_thetacontr.cc \
         conex/linear_solvers/conex/test/tune_hybrid.cc \
         conex/linear_solvers/conex/test/tune_qps.cc; do
  [ -f "$f" ] && git mv "$f" bench/
done

# 4. Move benchmark data
[ -d conex/linear_solvers/benchmark_data ] && git mv conex/linear_solvers/benchmark_data/* bench/data/ 2>/dev/null || true

# 5. Move docs (tex files only, skip build artifacts)
for f in conex/linear_solvers/doc/*.tex; do
  [ -f "$f" ] && git mv "$f" doc/
done

# 6. Move CMakeLists.txt to root (will need editing)
git mv conex/linear_solvers/CMakeLists.txt CMakeLists.txt.new

# 7. Move scripts
for f in conex/linear_solvers/run.sh \
         conex/linear_solvers/run_benchmarks.sh \
         conex/linear_solvers/profile_solver_construction.sh; do
  [ -f "$f" ] && git mv "$f" bench/
done
for f in conex/linear_solvers/coverage.sh \
         conex/linear_solvers/gpu_tests.sh \
         conex/linear_solvers/cleanup_report.sh \
         conex/linear_solvers/unused_code_report.sh; do
  [ -f "$f" ] && git mv "$f" scripts/
done

# 8. Move legacy code
# Legacy source
for f in conex/conex/*.cc conex/conex/*.h; do
  [ -f "$f" ] && [ "$(basename "$f")" != "linear_solvers" ] && git mv "$f" legacy/
done
# Legacy test
[ -d conex/conex/test ] && git mv conex/conex/test/* legacy/test/ 2>/dev/null || true
# Legacy Bazel
git mv conex/conex/BUILD legacy/BUILD.conex 2>/dev/null || true
git mv conex/conex/test.bzl legacy/ 2>/dev/null || true
# Interfaces
[ -d interfaces ] && git mv interfaces/* legacy/interfaces/ 2>/dev/null || true
# Examples
[ -d examples ] && git mv examples/* legacy/examples/ 2>/dev/null || true
# Third party
[ -d third_party ] && git mv third_party/* legacy/third_party/ 2>/dev/null || true
# Bazel root files
for f in BUILD WORKSPACE MODULE.bazel MODULE.bazel.lock build_all.sh; do
  [ -f "$f" ] && git mv "$f" legacy/
done
[ -d tools ] && git mv tools/* legacy/tools/ 2>/dev/null || true

echo "Restructure moves complete."
echo "Remaining in conex/linear_solvers/:"
ls conex/linear_solvers/ 2>/dev/null || echo "(empty or removed)"
echo ""
echo "New top-level:"
ls -d src tests bench doc legacy python scripts CMakeLists.txt.new 2>/dev/null
