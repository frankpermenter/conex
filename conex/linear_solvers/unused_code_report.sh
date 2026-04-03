#!/bin/bash
# Unused code detection using lcov merged coverage data.
# Builds in a clean git worktree, runs all tests, generates lcov report,
# then parses FNDA:0 entries to find functions never called.
# Usage: ./unused_code_report.sh [--skip-build]
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "$0")" && git rev-parse --show-toplevel)"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKTREE="/tmp/conex-coverage"
INFO_FILE="$SCRIPT_DIR/coverage_filtered.info"

SKIP_BUILD=false
for arg in "$@"; do
  case $arg in
    --skip-build) SKIP_BUILD=true ;;
  esac
done

if [ "$SKIP_BUILD" = false ]; then
  echo "Setting up clean worktree at $WORKTREE..." >&2
  cd "$REPO_ROOT"
  git worktree remove "$WORKTREE" 2>/dev/null || true
  git worktree add "$WORKTREE" HEAD 2>/dev/null

  BUILD_DIR="$WORKTREE/conex/linear_solvers"
  cd "$BUILD_DIR"

  echo "Building with coverage..." >&2
  cmake -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_FLAGS="--coverage -fprofile-arcs -ftest-coverage" . >/dev/null 2>&1
  make -j"$(nproc)" >/dev/null 2>&1

  echo "Running tests..." >&2
  lcov --zerocounters --directory . >/dev/null 2>&1
  ctest --output-on-failure >/dev/null 2>&1

  echo "Capturing coverage..." >&2
  lcov --capture --directory . --output-file coverage.info --no-external \
    --ignore-errors mismatch,negative >/dev/null 2>&1
  lcov --remove coverage.info '*/test/*' '*/_deps/*' '*/RLDLT.h' \
    --output-file "$INFO_FILE" \
    --ignore-errors mismatch,negative >/dev/null 2>&1
fi

cd "$SCRIPT_DIR"

if [ ! -f "$INFO_FILE" ]; then
  echo "No coverage file found: $INFO_FILE"
  echo "Run without --skip-build to generate coverage data."
  exit 1
fi

echo "# Unused Code Report"
echo ""
echo "Source: $INFO_FILE"
echo ""
echo "## Uncovered functions (never called)"
echo ""
echo "| Function | Source file |"
echo "|----------|------------|"

FOUND=false
CURRENT_FILE=""

while IFS= read -r line; do
  # Track current source file.
  if [[ "$line" == SF:* ]]; then
    CURRENT_FILE="${line#SF:}"
    # Only report conex/ sources, skip test/deps.
    if echo "$CURRENT_FILE" | grep -qE '/test/|/_deps/|RLDLT\.h'; then
      CURRENT_FILE=""
    fi
    continue
  fi

  # Skip if not in a conex source file.
  [ -z "$CURRENT_FILE" ] && continue

  # FNDA:count,name — function was called 'count' times.
  if [[ "$line" == FNDA:0,* ]]; then
    mangled="${line#FNDA:0,}"
    demangled=$(echo "$mangled" | c++filt 2>/dev/null)

    # Filter to conex:: namespace only.
    echo "$demangled" | grep -q 'conex::' || continue
    # Skip destructors, lambdas, template noise.
    echo "$demangled" | grep -qE '~|lambda|operator delete|__cxx' && continue
    # Skip PQTree — gcov false positive (verified live via canary test).
    echo "$demangled" | grep -q 'PQTree' && continue

    # Shorten for display.
    short=$(echo "$demangled" | sed 's/conex:://g; s/(anonymous namespace):://g')
    # Shorten the file path.
    file_short=$(echo "$CURRENT_FILE" | sed 's|.*/conex/|conex/|')

    echo "| \`${short}\` | \`${file_short}\` |"
    FOUND=true
  fi
done < "$INFO_FILE"

$FOUND || echo "| (none found) | |"

echo ""
echo "---"
echo "Generated: $(date -u '+%Y-%m-%d %H:%M UTC')"
