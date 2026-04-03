#!/bin/bash
# Unused code detection using lcov merged coverage data.
# Requires: coverage_filtered.info from a coverage build + test run.
# Usage: ./unused_code_report.sh [coverage_filtered.info]
set -uo pipefail
cd "$(dirname "$0")"

INFO_FILE="${1:-coverage_filtered.info}"

if [ ! -f "$INFO_FILE" ]; then
  echo "No coverage file found: $INFO_FILE"
  echo "Run cleanup_report.sh first to generate coverage data."
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
