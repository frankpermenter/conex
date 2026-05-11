#!/bin/bash
# Profile solver construction using perf.
# Usage: ./profile_solver_construction.sh [mtx_file]
# Output: perf flat profile (self time, build only)
set -uo pipefail
cd "$(dirname "$0")"

BUILD_DIR="./build"
PROFILE_MTX="$BUILD_DIR/profile_mtx"
DEFAULT_MTX="/agent-workspace/interfaces/python/test/benchmark_data/bcsstk16/bcsstk16.mtx"

MTX_FILE="${1:-$DEFAULT_MTX}"

if [ ! -f "$MTX_FILE" ]; then
  echo "MTX file not found: $MTX_FILE" >&2
  exit 1
fi

# Always rebuild to avoid stale binaries.
echo "Building profile_mtx..." >&2
mkdir -p "$BUILD_DIR"
cmake -S . -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CXX_FLAGS="-fno-omit-frame-pointer -g" \
  >/dev/null 2>&1
cmake --build "$BUILD_DIR" -j"$(nproc)" --target profile_mtx >/dev/null 2>&1

# Check perf permissions.
PARANOID=$(cat /proc/sys/kernel/perf_event_paranoid 2>/dev/null || echo 0)
if [ "$PARANOID" -gt 1 ]; then
  echo "perf_event_paranoid=$PARANOID (need <=1). Trying sudo..." >&2
  sudo sysctl -w kernel.perf_event_paranoid=1 >/dev/null 2>&1 || {
    echo "Cannot lower perf_event_paranoid." >&2
    echo "Run: sudo sysctl -w kernel.perf_event_paranoid=1" >&2
    exit 1
  }
fi

echo "Profiling: $(basename "$MTX_FILE") (build only, --iters 0)" >&2

PERF_DATA=$(mktemp /tmp/perf.XXXXXX.data)
trap "rm -f '$PERF_DATA'" EXIT

perf record -g --call-graph dwarf -o "$PERF_DATA" \
  "$PROFILE_MTX" --randomize --iters 0 "$MTX_FILE" 2>/dev/null

perf report -i "$PERF_DATA" --stdio --no-children \
  --percent-limit 0.8 --sort=dso,sym 2>/dev/null | cat
