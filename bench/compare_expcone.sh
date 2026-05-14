#!/bin/bash
# Compare conex vs SCS on random exponential cone problems.
# Usage: ./compare_expcone.sh [m] [p] [seed]

M="${1:-10}"
P="${2:-6}"
SEED="${3:-42}"
DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "=== Exp cone comparison: m=$M, p=$P, seed=$SEED ==="
echo

echo "--- Conex ---"
"$DIR/build/benchmark_expcone" "$M" "$P" "$SEED"
echo

echo "--- SCS ---"
"$DIR/build/benchmark_expcone" "$M" "$P" "$SEED" --dump | python3 "$DIR/bench/compare_expcone.py"
