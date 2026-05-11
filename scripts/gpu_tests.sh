#!/bin/bash
set -e
cd "$(dirname "$0")"

rm -f CMakeCache.txt
cmake . -DCMAKE_BUILD_TYPE=Release

if ! grep -q "gpu_tree_solver" CMakeCache.txt 2>/dev/null && \
   ! cmake --build . --target help 2>/dev/null | grep -q gpu_tree_solver_test; then
  echo "CUDA not found — cannot build GPU tests."
  exit 1
fi

make -j$(nproc) gpu_tree_solver_test
./gpu_tree_solver_test
