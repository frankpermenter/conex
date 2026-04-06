#!/bin/bash
# Cleanup report: build, test, benchmark, and find unused code.
# Usage: ./cleanup_report.sh [--skip-build] [--skip-bench] [--skip-coverage]
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR" && git rev-parse --show-toplevel 2>/dev/null || echo "$SCRIPT_DIR")"
cd "$SCRIPT_DIR"

SKIP_BUILD=false
SKIP_BENCH=false
SKIP_COVERAGE=false
for arg in "$@"; do
  case $arg in
    --skip-build) SKIP_BUILD=true ;;
    --skip-bench) SKIP_BENCH=true ;;
    --skip-coverage) SKIP_COVERAGE=true ;;
  esac
done

# ===================================================================
# 1. Build and test
# ===================================================================
if [ "$SKIP_BUILD" = false ]; then
  cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="" . >/dev/null 2>&1
  make -j"$(nproc)" >/dev/null 2>&1
fi

TEST_OUTPUT=$(ctest --output-on-failure 2>&1)
# ctest summary: "100% tests passed, 0 tests failed out of 23"
SUMMARY_LINE=$(echo "$TEST_OUTPUT" | grep 'failed out of' || echo "0 tests passed, 0 tests failed out of 0")
TOTAL=$(echo "$SUMMARY_LINE" | sed -E 's/.*out of ([0-9]+).*/\1/')
FAILED=$(echo "$SUMMARY_LINE" | sed -E 's/.*,\s*([0-9]+) tests? failed.*/\1/')
PASSED=$((TOTAL - FAILED))

# Parse per-suite results.
declare -A SUITE_PASS SUITE_FAIL
while IFS= read -r line; do
  if echo "$line" | grep -qE 'Test[[:space:]]+#[0-9]+:'; then
    name=$(echo "$line" | sed -E 's/.*Test[[:space:]]+#[0-9]+:[[:space:]]*//' | sed -E 's/[[:space:]]+\.+.*//')
    suite=$(echo "$name" | sed -E 's/\..*//')
    if echo "$line" | grep -q 'Passed'; then
      SUITE_PASS[$suite]=$(( ${SUITE_PASS[$suite]:-0} + 1 ))
    else
      SUITE_FAIL[$suite]=$(( ${SUITE_FAIL[$suite]:-0} + 1 ))
    fi
  fi
done <<< "$TEST_OUTPUT"

# ===================================================================
# 2. Benchmarks
# ===================================================================
BENCH_OUTPUT=""
if [ "$SKIP_BENCH" = false ] && [ -x ./profile_mtx ]; then
  # Discover .mtx files in common locations.
  MTX_FILES=""
  for dir in \
    /agent-workspace/interfaces/python/test/benchmark_data \
    "$SCRIPT_DIR/benchmark_data" \
    "$REPO_ROOT/benchmark_data"; do
    if [ -d "$dir" ]; then
      for f in "$dir"/*/*.mtx; do
        [ -f "$f" ] && MTX_FILES="$MTX_FILES $f"
      done
    fi
  done
  if [ -n "$MTX_FILES" ]; then
    BENCH_OUTPUT=$(./profile_mtx --randomize $MTX_FILES 2>&1)
  fi
fi

# ===================================================================
# 2b. Coverage (via unused_code_report.sh — Clang/llvm-cov)
# ===================================================================
COVERAGE_OUTPUT=""
if [ "$SKIP_COVERAGE" = false ]; then
  COVERAGE_OUTPUT=$(bash "$SCRIPT_DIR/unused_code_report.sh")
fi

# ===================================================================
# 3. Unused code detection
# ===================================================================
SRC_DIR=conex

# 3a. Unused classes: declared in .h, never referenced in other files.
# Also checks for internal usage: using aliases, member types, base classes
# within the same file (beyond the class declaration line itself).
UNUSED_CLASSES=""
while IFS= read -r match; do
  cls=$(echo "$match" | sed -E 's/.*class ([A-Z][A-Za-z_0-9]*).*/\1/')
  decl_file=$(echo "$match" | cut -d: -f1)
  decl_line=$(echo "$match" | cut -d: -f2)
  # Count files referencing this class (excluding declaration file).
  ref_count=$(grep -rl "\b${cls}\b" "$SRC_DIR" --include='*.h' --include='*.cc' 2>/dev/null \
    | grep -v "$decl_file" | wc -l)
  # Check for internal usage in the same file (excluding the declaration line
  # and comments). Catches member types, base classes, using aliases.
  internal_uses=$(grep -n "\b${cls}\b" "$decl_file" 2>/dev/null \
    | grep -v "^${decl_line}:" \
    | grep -v '^\s*//' \
    | grep -v '^[0-9]*:\s*//' \
    | grep -v '#if 0' \
    | wc -l)
  if [ "$ref_count" -eq 0 ] && [ "$internal_uses" -le 0 ]; then
    line_num=$decl_line
    UNUSED_CLASSES="${UNUSED_CLASSES}| \`${cls}\` | \`${decl_file}:${line_num}\` | No references outside declaring file |\n"
  fi
done < <(grep -rn '^class [A-Z]' "$SRC_DIR" --include='*.h' | grep -v '//' | grep -v 'template')

# 3b. #if 0 blocks.
IF0_BLOCKS=""
while IFS= read -r match; do
  file=$(echo "$match" | cut -d: -f1)
  line=$(echo "$match" | cut -d: -f2)
  # Try to get a description from the next non-empty line or comment.
  desc=$(sed -n "$((line+1))p" "$file" | sed 's/^[[:space:]]*//' | head -c 60)
  [ -z "$desc" ] && desc="dead code block"
  IF0_BLOCKS="${IF0_BLOCKS}| \`${file}:${line}\` | ${desc} |\n"
done < <(grep -rn '#if 0' "$SRC_DIR" --include='*.h' --include='*.cc' 2>/dev/null)

# 3c. Commented-out using/template aliases.
COMMENTED=""
while IFS= read -r match; do
  file=$(echo "$match" | cut -d: -f1)
  line=$(echo "$match" | cut -d: -f2)
  content=$(echo "$match" | cut -d: -f3- | sed 's/^[[:space:]]*//')
  COMMENTED="${COMMENTED}| \`${file}:${line}\` | \`${content}\` |\n"
done < <(grep -rn '^\s*//\s*\(using\|template\)' "$SRC_DIR" --include='*.h' --include='*.cc' 2>/dev/null)

# 3d. Unused methods: AddSupernode, AddSeparator, set_variable_indices, variable_indices_.
UNUSED_MEMBERS=""
for name in AddSupernode AddSeparator; do
  decl=$(grep -rn "\b${name}\b" "$SRC_DIR" --include='*.h' | head -1)
  if [ -n "$decl" ]; then
    decl_file=$(echo "$decl" | cut -d: -f1)
    usage=$(grep -rn "\b${name}\b" "$SRC_DIR" --include='*.h' --include='*.cc' 2>/dev/null \
      | grep -v "$decl_file" | grep -v '^\s*//' | wc -l)
    if [ "$usage" -eq 0 ]; then
      line_num=$(echo "$decl" | cut -d: -f2)
      UNUSED_MEMBERS="${UNUSED_MEMBERS}| \`${name}\` | \`${decl_file}:${line_num}\` | Never called |\n"
    fi
  fi
done

# ===================================================================
# 4. Format report
# ===================================================================
echo ""
echo "# Cleanup Report"
echo ""
echo "## 1. Test Results"
echo ""
echo "**${PASSED}/${TOTAL} tests passed** (${FAILED} failures)"
echo ""
echo "| Suite | Tests | Status |"
echo "|-------|-------|--------|"
for suite in $(echo "${!SUITE_PASS[@]} ${!SUITE_FAIL[@]}" | tr ' ' '\n' | sort -u); do
  p=${SUITE_PASS[$suite]:-0}
  f=${SUITE_FAIL[$suite]:-0}
  total=$((p + f))
  if [ "$f" -gt 0 ]; then
    echo "| ${suite} | ${total} | **FAIL** (${f} failed) |"
  else
    echo "| ${suite} | ${total} | PASS |"
  fi
done

if [ "$FAILED" -gt 0 ]; then
  echo ""
  echo "### Failures"
  echo '```'
  echo "$TEST_OUTPUT" | grep -A2 'FAILED\|Failure'
  echo '```'
fi

echo ""
echo "## 2. Benchmark Results"
echo ""
if [ -n "$BENCH_OUTPUT" ]; then
  echo "| Matrix | asm+fac | solve | residual |"
  echo "|--------|---------|-------|----------|"
  echo "$BENCH_OUTPUT" | grep -E '^[a-zA-Z]' | grep -v '^Matrix\|^Col\|^Sta' | while IFS= read -r line; do
    name=$(echo "$line" | awk '{print $1}')
    # Extract fields by position: columns are fixed-width.
    # asm+fac and solve are the 5th and 6th "Xus" fields.
    asmfac=$(echo "$line" | grep -oE '[0-9]+us' | sed -n '5p')
    solve=$(echo "$line" | grep -oE '[0-9]+us' | sed -n '6p')
    resid=$(echo "$line" | grep -oE '[0-9]+\.[0-9]+e[+-][0-9]+' | tail -1)
    if [ -n "$asmfac" ] && [ -n "$resid" ]; then
      echo "| ${name} | ${asmfac} | ${solve} | ${resid} |"
    elif echo "$line" | grep -q "FACTOR FAILED"; then
      echo "| ${name} | FACTOR FAILED | - | rank-def |"
    fi
  done
else
  echo "(benchmarks skipped)"
fi

if [ -n "$COVERAGE_OUTPUT" ]; then
  echo ""
  echo "$COVERAGE_OUTPUT"
  echo ""
fi

echo ""
echo "## 3. Unused Code"
echo ""

echo "### Dead classes"
echo "| Class | File | Notes |"
echo "|-------|------|-------|"
if [ -n "$UNUSED_CLASSES" ]; then
  echo -e "$UNUSED_CLASSES"
else
  echo "| (none found) | | |"
fi

echo ""
echo "### Dead code blocks"
echo "| Location | Description |"
echo "|----------|-------------|"
if [ -n "$IF0_BLOCKS" ]; then
  echo -e "$IF0_BLOCKS"
fi
if [ -n "$COMMENTED" ]; then
  echo -e "$COMMENTED"
fi
if [ -z "$IF0_BLOCKS" ] && [ -z "$COMMENTED" ]; then
  echo "| (none found) | |"
fi

echo ""
echo "### Unused members/methods"
echo "| Item | File | Notes |"
echo "|------|------|-------|"
if [ -n "$UNUSED_MEMBERS" ]; then
  echo -e "$UNUSED_MEMBERS"
else
  echo "| (none found) | | |"
fi

echo ""
echo "## 4. Recommendations"
echo ""
echo "**Safe to remove:**"
{
  if [ -n "$UNUSED_CLASSES" ]; then
    echo -e "$UNUSED_CLASSES" | while IFS='|' read -r _ cls _ _; do
      cls=$(echo "$cls" | sed 's/`//g' | xargs)
      [ -n "$cls" ] && echo "Remove \`${cls}\`"
    done
  fi
  if [ -n "$IF0_BLOCKS" ]; then
    echo "Remove all \`#if 0\` blocks"
  fi
  if [ -n "$COMMENTED" ]; then
    echo "Remove commented-out using/template aliases"
  fi
  if [ -n "$UNUSED_MEMBERS" ]; then
    echo -e "$UNUSED_MEMBERS" | while IFS='|' read -r _ item _ _; do
      item=$(echo "$item" | sed 's/`//g' | xargs)
      [ -n "$item" ] && echo "Remove \`${item}\`"
    done
  fi
} | awk '{print NR". "$0}'

# Check agents.md lists all algorithm source files.
AGENTS_MD="conex/algorithms/agents.md"
MISSING_FROM_DOC=""
if [ -f "$AGENTS_MD" ]; then
  for src in conex/algorithms/*.cc; do
    base=$(basename "$src")
    if ! grep -q "$base" "$AGENTS_MD" 2>/dev/null; then
      MISSING_FROM_DOC="${MISSING_FROM_DOC}${base} "
    fi
  done
fi
if [ -n "$MISSING_FROM_DOC" ]; then
  echo ""
  echo "**agents.md out of sync:** missing ${MISSING_FROM_DOC}"
fi

echo ""
echo "---"
echo "Generated: $(date -u '+%Y-%m-%d %H:%M UTC')"
