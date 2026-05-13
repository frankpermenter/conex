#!/bin/bash
# Compare conex and Clarabel on an SDPA file.
# Usage: ./compare.sh <file.dat-s> [conex_algorithm]
#
# Example: ./compare.sh /agent-workspace/problem_libraries/SDPLIB/data/truss5.dat-s thetacontr

set -e

FILE="$1"
ALG="${2:-lp}"

if [ -z "$FILE" ]; then
  echo "Usage: $0 <file.dat-s> [lp|barrierlp|thetacont|thetacontr]"
  exit 1
fi

NAME=$(basename "$FILE" .dat-s)
CONEX="$(dirname "$0")/../build/timing_breakdown"
CLARABEL="/tmp/clarabel_build/release/examples/benchmark_sdp"

echo "=== $NAME ==="
echo ""

# Conex
echo "--- Conex ($ALG) ---"
$CONEX "$FILE" "$ALG" 2>&1 | grep -v "^$"
echo ""

# Clarabel
echo "--- Clarabel ---"
$CLARABEL "$FILE" 100 2>&1 | grep -E "problem:|variables|constraints|nnz|cones|linear algebra|Terminated|Solve:|=== |Read:|Setup:|Total:|kkt|scale|IP iter|default|setup|post"
