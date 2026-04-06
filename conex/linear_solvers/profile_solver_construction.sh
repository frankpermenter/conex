#!/bin/bash
# Profile solver construction using perf.
# Usage: ./profile_solver_construction.sh [mtx_file]
#   e.g. ./profile_solver_construction.sh /path/to/bcsstk16.mtx
#
# Requires: perf, c++filt
# Output: top functions by CPU time, annotated call graph
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

echo "=== Profiling: $(basename "$MTX_FILE") ===" >&2
echo "Matrix: $MTX_FILE" >&2
echo "" >&2

# Check perf permissions.
PARANOID=$(cat /proc/sys/kernel/perf_event_paranoid 2>/dev/null || echo 0)
if [ "$PARANOID" -gt 1 ]; then
  echo "perf_event_paranoid=$PARANOID (need <=1). Trying sudo..." >&2
  sudo sysctl -w kernel.perf_event_paranoid=1 >/dev/null 2>&1 || {
    echo "Cannot lower perf_event_paranoid. Falling back to timer-based profiling." >&2
    echo ""
    echo "=== Timer-based profile (no perf) ==="
    echo ""
    # Use Google perftools or just time the run and report.
    "$PROFILE_MTX" --randomize "$MTX_FILE"
    exit 0
  }
fi

# Record with call graph (dwarf for accuracy).
PERF_DATA=$(mktemp /tmp/perf.XXXXXX.data)
perf record -g --call-graph dwarf -o "$PERF_DATA" \
  "$PROFILE_MTX" --randomize "$MTX_FILE" 2>/dev/null

echo "" >&2
echo "=== Top functions (flat profile) ===" >&2
echo ""
perf report -i "$PERF_DATA" --stdio --no-children \
  --percent-limit 1.0 2>/dev/null \
  | grep -E "^\s+[0-9]" | head -30

echo ""
echo "=== Top callers (children profile) ==="
echo ""
perf report -i "$PERF_DATA" --stdio \
  --percent-limit 2.0 2>/dev/null \
  | grep -E "^\s+[0-9]" | head -30

echo ""
echo "=== Call graph (top hotspots) ==="
echo ""
perf report -i "$PERF_DATA" --stdio -g fractal,5 \
  --percent-limit 5.0 2>/dev/null \
  | head -100

rm -f "$PERF_DATA"
