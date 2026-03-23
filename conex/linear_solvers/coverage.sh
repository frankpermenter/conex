#!/bin/bash
# Generate a coverage report for the linear_solvers project.
# Usage: ./coverage.sh [build_dir]
#   build_dir defaults to "build" relative to this script's directory.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${1:-$SCRIPT_DIR/build}"

# Clean old coverage data.
find "$BUILD_DIR" -name "*.gcda" -delete 2>/dev/null || true
find "$BUILD_DIR" -name "*.gcno" -delete 2>/dev/null || true

# Rebuild with coverage flags.
cd "$BUILD_DIR"
cmake "$SCRIPT_DIR" -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="--coverage" \
  -DCMAKE_EXE_LINKER_FLAGS="--coverage" > /dev/null 2>&1
make clean > /dev/null 2>&1
make -j"$(nproc)" 2>&1 | tail -1

# Run tests to generate coverage data.
./sparse_linear_constraint_test 2>&1 | grep -E "^\[  (PASSED|FAILED)"
./multithreading_test 2>&1 | grep -E "^\[  (PASSED|FAILED)"

# Collect coverage for project source files.
echo ""
echo "=== Coverage Report ==="
printf "%-45s %10s %8s\n" "File" "Coverage" "Lines"
printf "%-45s %10s %8s\n" "----" "--------" "-----"

for gcda in $(find "$BUILD_DIR" -path "*/_deps" -prune -o -name "*.gcda" -print | sort); do
  dir=$(dirname "$gcda")
  gcov -n -o "$dir" "$gcda" 2>/dev/null \
    | grep -A1 "^File '.*linear_solvers/conex/.*\.cc'" \
    | grep -E "^(File|Lines)"
done \
  | paste - - \
  | sed "s|File '${SCRIPT_DIR}/||;s|'||g" \
  | while IFS=$'\t' read -r file lines; do
      pct=$(echo "$lines" | grep -oP '[\d.]+%')
      cnt=$(echo "$lines" | grep -oP 'of \K\d+')
      pct_num=$(echo "$pct" | tr -d '%')
      printf "%s\t%s\t%s\t%s\n" "$pct_num" "$file" "$pct" "$cnt"
    done \
  | sort -t$'\t' -k1 -rn \
  | while IFS=$'\t' read -r _ file pct cnt; do
      printf "%-45s %10s %8s\n" "$file" "$pct" "$cnt"
    done

echo ""
