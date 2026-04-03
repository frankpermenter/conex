#!/bin/bash
# Unused code detection using Clang source-based coverage.
# Builds in a clean git worktree, runs tests, reports uncovered functions.
# Usage: ./unused_code_report.sh [--skip-build]
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "$0")" && git rev-parse --show-toplevel)"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKTREE="/tmp/conex-coverage"
PROFDATA="$WORKTREE/conex/linear_solvers/merged.profdata"

SKIP_BUILD=false
for arg in "$@"; do
  case $arg in
    --skip-build) SKIP_BUILD=true ;;
  esac
done

if [ "$SKIP_BUILD" = false ]; then
  echo "Setting up clean worktree..." >&2
  cd "$REPO_ROOT"
  git worktree remove --force "$WORKTREE" 2>/dev/null || true
  git worktree add "$WORKTREE" HEAD

  BUILD_DIR="$WORKTREE/conex/linear_solvers"
  cd "$BUILD_DIR"

  echo "Building with Clang coverage..." >&2
  rm -rf CMakeCache.txt CMakeFiles
  cmake -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_C_COMPILER=clang \
    -DCMAKE_CXX_COMPILER=clang++ \
    -DCMAKE_CXX_FLAGS="-fprofile-instr-generate -fcoverage-mapping" \
    -DCMAKE_EXE_LINKER_FLAGS="-fprofile-instr-generate" \
    -DCMAKE_CUDA_COMPILER=NOTFOUND \
    . >/dev/null 2>&1
  TEST_TARGETS=$(grep -oP '(?<=add_executable\()[\w]+_test' CMakeLists.txt | tr '\n' ' ')
  echo "Building: $TEST_TARGETS" >&2
  make -k -j"$(nproc)" $TEST_TARGETS 2>&1 | tail -1 >&2

  echo "Running tests..." >&2
  mkdir -p profraw
  for bin in ./*_test; do
    [ -x "$bin" ] || continue
    echo "  $(basename "$bin")" >&2
    LLVM_PROFILE_FILE="profraw/$(basename "$bin").profraw" \
      "$bin" >/dev/null 2>&1 || true
  done

  echo "Merging $(ls profraw/*.profraw 2>/dev/null | wc -l) profiles..." >&2
  llvm-profdata merge -sparse profraw/*.profraw -o "$PROFDATA" 2>/dev/null
fi

BUILD_DIR="$WORKTREE/conex/linear_solvers"
cd "$BUILD_DIR"

if [ ! -f "$PROFDATA" ]; then
  echo "No profile data found. Run without --skip-build first."
  exit 1
fi

# Collect all test binaries as -object args.
OBJECTS=""
for bin in ./*_test; do
  [ -x "$bin" ] && OBJECTS="$OBJECTS -object=$bin"
done

echo "# Unused Code Report (llvm-cov)"
echo ""

# Test binary inventory.
echo "## Test binaries"
echo ""
echo "| Binary | Profiled |"
echo "|--------|----------|"
for bin in ./*_test; do
  name=$(basename "$bin")
  if [ -f "profraw/${name}.profraw" ]; then
    echo "| \`${name}\` | yes |"
  elif [ -x "$bin" ]; then
    echo "| \`${name}\` | **no** (ran but no profile) |"
  else
    echo "| \`${name}\` | **no** (not built) |"
  fi
done
PROFILE_COUNT=$(ls profraw/*.profraw 2>/dev/null | wc -l)
echo ""
echo "Profiles merged: ${PROFILE_COUNT}"
echo ""

# File-level coverage summary (native llvm-cov output, no parsing).
echo "## File coverage summary"
echo ""
echo '```'
llvm-cov report $OBJECTS -instr-profile="$PROFDATA" 2>/dev/null | grep -E "^(Filename|------|conex/|TOTAL)"
echo '```'
echo ""

# Uncovered function names require the JSON export + demangling.
echo "## Uncovered functions (0 execution count)"
echo ""
echo "| Function | Source file |"
echo "|----------|------------|"

llvm-cov export $OBJECTS -instr-profile="$PROFDATA" 2>/dev/null | python3 -c "
import json, sys, subprocess

data = json.load(sys.stdin)
for file_data in data.get('data', []):
    for fn in file_data.get('functions', []):
        if fn.get('count', 1) != 0:
            continue
        filenames = fn.get('filenames', [])
        if not filenames:
            continue
        fname = filenames[0]
        if '/test/' in fname or '/_deps/' in fname or 'RLDLT.h' in fname:
            continue
        if '/conex/' not in fname:
            continue

        mangled = fn.get('name', '')
        try:
            result = subprocess.run(['c++filt', mangled], capture_output=True, text=True, timeout=1)
            name = result.stdout.strip()
        except:
            name = mangled

        if 'conex::' not in name:
            continue
        if any(s in name for s in ['~', 'lambda', 'operator delete', '__cxx']):
            continue

        short = name.replace('conex::', '').replace('(anonymous namespace)::', '')
        file_short = fname.split('/conex/')[-1] if '/conex/' in fname else fname

        print(f'| \`{short}\` | \`conex/{file_short}\` |')
"

echo ""
echo "---"
echo "Generated: $(date -u '+%Y-%m-%d %H:%M UTC')"
