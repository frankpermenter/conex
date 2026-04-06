#!/bin/bash
# Profile solver construction using perf.
# Usage: ./profile_solver_construction.sh [mtx_file]
#   e.g. ./profile_solver_construction.sh /path/to/bcsstk16.mtx
#
# Requires: perf
# Output: clean flat profile table with demangled short names
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

# Record.
PERF_DATA=$(mktemp /tmp/perf.XXXXXX.data)
perf record -g --call-graph dwarf -o "$PERF_DATA" \
  "$PROFILE_MTX" --randomize --iters 0 "$MTX_FILE" 2>/dev/null

# Extract flat profile and clean up names.
echo ""
echo "=== Flat profile (self time, build only) ==="
echo ""
printf "  %6s  %-60s  %s\n" "Self%" "Function" "Library"
printf "  %6s  %-60s  %s\n" "------" "------------------------------------------------------------" "-------"

perf report -i "$PERF_DATA" --stdio --no-children \
  --percent-limit 0.8 2>/dev/null \
  | grep -E "^\s+[0-9]+\.[0-9]+%" \
  | head -20 \
  | while IFS= read -r line; do
    pct=$(echo "$line" | awk '{print $1}')
    lib=$(echo "$line" | awk '{print $3}')
    # Extract the symbol (everything after [.] or [k]).
    sym=$(echo "$line" | sed 's/.*\[\.\] //' | sed 's/.*\[k\] /kernel:/')
    # Shorten C++ template noise to readable names.
    sym=$(echo "$sym" | sed '
      s/ \[clone [^]]*\]//g
      s/conex::(anonymous namespace):://g
      s/conex:://g
    ')
    # Strip all template arguments: Foo<...> → Foo
    sym=$(echo "$sym" | python3 -c "
import sys, re
s = sys.stdin.read().strip()
# Repeatedly strip innermost <...> until none left.
while '<' in s:
    s = re.sub(r'<[^<>]*>', '', s)
# Clean up artifacts.
s = s.replace('std::__introsort_loop', 'std::sort')
s = s.replace('std::__detail::_Map_base::operator[]', 'unordered_map[]')
s = s.replace('std::num_get::_M_extract_float', 'strtod')
s = s.replace('std::num_get::_M_extract_int', 'strtoi')
s = s.replace('Eigen::internal::gebp_kernel::operator()', 'Eigen::gebp_kernel')
s = s.replace('Eigen::internal::general_matrix_vector_product::run', 'Eigen::gemv')
s = s.replace('Eigen::internal::triangular_solve_matrix::run', 'Eigen::trsm')
s = s.replace('Eigen::internal::dense_assignment_loop::run', 'Eigen::assign')
s = s.replace('std::istreambuf_iterator', 'istream_iter')
s = s.replace('std::istream::sentry::sentry', 'istream::sentry')
print(s[:60])
" 2>/dev/null || echo "$sym" | cut -c1-60)
    # Shorten library name.
    lib=$(echo "$lib" | sed 's/libopenblasp-r0.3.26.so/openblas/' | sed 's/libstdc++.so.6.0.33/libstdc++/')
    printf "  %6s  %-60s  %s\n" "$pct" "$sym" "$lib"
  done

echo ""
rm -f "$PERF_DATA"
