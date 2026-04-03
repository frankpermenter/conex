#!/bin/bash
# Unused code detection using gcov function-level coverage data.
# Requires: gcda files from a coverage build + test run.
# Usage: ./unused_code_report.sh
set -uo pipefail
cd "$(dirname "$0")"

SRC_DIR=conex

echo "# Unused Code Report (gcov-based)"
echo ""

# Collect all gcda files for our source (not deps/tests).
GCDA_FILES=$(find . -name '*.gcda' -path '*/CMakeFiles/*' \
  | grep -v '_deps' | grep -v 'test')

if [ -z "$GCDA_FILES" ]; then
  echo "No gcda files found. Run tests with a --coverage build first."
  exit 1
fi

echo "## Uncovered functions (0% line execution)"
echo ""
echo "Functions in conex/ source that were never called during tests."
echo ""
echo "| Function | Source file |"
echo "|----------|------------|"

FOUND=false
for gcda in $GCDA_FILES; do
  # Extract source file name from gcda path.
  src=$(echo "$gcda" | sed 's|.*/CMakeFiles/[^/]*/||; s|\.gcda||; s|\.cc\.||')

  # Run gcov -f to get per-function coverage.
  gcov -f "$gcda" 2>/dev/null | \
    grep -B1 "Lines executed:0.00%" | \
    grep "^Function" | \
    while read -r line; do
      # Extract mangled name, demangle, filter to conex:: only.
      mangled=$(echo "$line" | sed "s/^Function '//; s/'$//")
      demangled=$(echo "$mangled" | c++filt 2>/dev/null)

      # Skip non-conex functions (std::, Eigen::, etc.)
      echo "$demangled" | grep -q 'conex::' || continue
      # Skip destructors, lambda internals, template noise.
      echo "$demangled" | grep -qE '~|lambda|operator delete|__' && continue
      # Skip template instantiations of std:: containers.
      echo "$demangled" | grep -qE 'std::|Eigen::' && continue

      # Clean up for display.
      short=$(echo "$demangled" | sed 's/conex:://g; s/(anonymous namespace):://g')
      echo "| \`${short}\` | ${src} |"
      FOUND=true
    done
done

$FOUND || echo "| (none found) | |"

echo ""
echo "---"
echo "Generated: $(date -u '+%Y-%m-%d %H:%M UTC')"
