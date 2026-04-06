#!/bin/bash
# Run profile_mtx on SuiteSparse benchmark matrices.
# Usage: ./run_benchmarks.sh [profile_mtx options...]
#   e.g. ./run_benchmarks.sh --randomize
#        ./run_benchmarks.sh --randomize --sweep-threads 1,2,4
set -uo pipefail
cd "$(dirname "$0")"

MTX_FILES=(
  /agent-workspace/interfaces/python/test/benchmark_data/ash958/ash958.mtx
  /agent-workspace/interfaces/python/test/benchmark_data/illc1033/illc1033.mtx
  /agent-workspace/interfaces/python/test/benchmark_data/illc1850/illc1850.mtx
  /agent-workspace/interfaces/python/test/benchmark_data/well1033/well1033.mtx
  /agent-workspace/interfaces/python/test/benchmark_data/well1850/well1850.mtx
  /agent-workspace/interfaces/python/test/benchmark_data/aircraft/aircraft.mtx
  /agent-workspace/interfaces/python/test/benchmark_data/lp_fit2p/lp_fit2p.mtx
  /agent-workspace/interfaces/python/test/benchmark_data/p0201/p0201.mtx
  /agent-workspace/interfaces/python/test/benchmark_data/ash292/ash292.mtx
  /agent-workspace/interfaces/python/test/benchmark_data/bcsstk13/bcsstk13.mtx
  /agent-workspace/interfaces/python/test/benchmark_data/bcsstk14/bcsstk14.mtx
  /agent-workspace/interfaces/python/test/benchmark_data/bcsstk16/bcsstk16.mtx
  /agent-workspace/interfaces/python/test/benchmark_data/lshp3466/lshp3466.mtx
  /agent-workspace/interfaces/python/test/benchmark_data/west2021/west2021.mtx
  /agent-workspace/interfaces/python/test/benchmark_data/steam3/steam3.mtx
  /agent-workspace/interfaces/python/test/benchmark_data/orsirr_1/orsirr_1.mtx
  /agent-workspace/interfaces/python/test/benchmark_data/rdb3200l/rdb3200l.mtx
)

if [ ! -x ./profile_mtx ]; then
  echo "Building profile_mtx..." >&2
  make -j"$(nproc)" profile_mtx >/dev/null 2>&1
fi

# Filter to files that exist.
FOUND=()
MISSING=()
for f in "${MTX_FILES[@]}"; do
  if [ -f "$f" ]; then
    FOUND+=("$f")
  else
    MISSING+=("$(basename "$(dirname "$f")")")
  fi
done

if [ ${#MISSING[@]} -gt 0 ]; then
  echo "Missing: ${MISSING[*]}" >&2
fi

if [ ${#FOUND[@]} -eq 0 ]; then
  echo "No .mtx files found. Download with:" >&2
  echo '  cd /agent-workspace/interfaces/python/test/benchmark_data' >&2
  echo '  for name in ash958 illc1850 well1850; do' >&2
  echo '    curl -sL "https://suitesparse-collection-website.herokuapp.com/MM/HB/${name}.tar.gz" | tar xz' >&2
  echo '  done' >&2
  exit 1
fi

echo "Running ${#FOUND[@]} matrices..." >&2
./profile_mtx "$@" "${FOUND[@]}"
