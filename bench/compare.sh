#!/bin/bash
# Compare conex and Clarabel on an SDPA file.
# Usage: ./compare.sh <file.dat-s> [conex_algorithm]

FILE="$1"
ALG="${2:-lp}"

if [ -z "$FILE" ]; then
  echo "Usage: $0 <file.dat-s> [lp|barrierlp|thetacont|thetacontr]"
  exit 1
fi

NAME=$(basename "$FILE" .dat-s)
CONEX="$(cd "$(dirname "$0")/.." && pwd)/build/timing_breakdown"
CLARABEL="/tmp/clarabel_build/release/examples/benchmark_sdp"

to_us() {
  local val="$1"
  if echo "$val" | grep -q 'µs$'; then
    echo "$val" | sed 's/µs$//'
  elif echo "$val" | grep -q 'ms$'; then
    echo "$val" | sed 's/ms$//' | awk '{printf "%.0f", $1 * 1000}'
  elif echo "$val" | grep -q 's$'; then
    echo "$val" | sed 's/s$//' | awk '{printf "%.0f", $1 * 1000000}'
  else
    echo "$val"
  fi
}

div() { echo "$1 $2" | awk '{if($2>0) printf "%.0f", $1/$2; else print "—"}'; }

# Run conex
conex_out=$($CONEX "$FILE" "$ALG" 2>&1)
c_total=$(echo "$conex_out" | grep "Total solve:" | awk '{print $3}')
c_fac=$(echo "$conex_out" | grep "  Factor:" | awk '{print $2}')
c_fac_n=$(echo "$conex_out" | grep "  Factor:" | sed 's/.*(\([0-9]*\) calls.*/\1/')
c_solve=$(echo "$conex_out" | grep "  Solve:" | awk '{print $2}')
c_solve_n=$(echo "$conex_out" | grep "  Solve:" | sed 's/.*(\([0-9]*\) calls.*/\1/')
c_cone=$(echo "$conex_out" | grep "Cone ops:" | awk '{print $3}')
c_select=$(echo "$conex_out" | grep "Selection:" | awk '{print $2}')
c_iter=$(echo "$conex_out" | grep "Total solve:" | sed 's/.*(\([0-9]*\) iter.*/\1/')
c_nfac=$(echo "$conex_out" | grep "Total solve:" | sed 's/.*, \([0-9]*\) fac.*/\1/')
c_obj=$(echo "$conex_out" | grep "Objective:" | awk '{print $2}')
c_gap=$(echo "$conex_out" | grep "^Gap:" | awk '{print $2}')
c_build=$(echo "$conex_out" | grep "Build:" | awk '{print $2}')

# Run clarabel
clar_out=$($CLARABEL "$FILE" 100 2>&1)
k_total_ms=$(echo "$clar_out" | grep "^Solve:" | awk '{print $2}')
k_total=$(echo "$k_total_ms" | awk '{printf "%.0f", $1 * 1000}')
k_fac=$(to_us "$(echo "$clar_out" | grep "kkt update" | awk '{print $NF}')")
k_solve=$(to_us "$(echo "$clar_out" | grep "kkt solve" | awk '{print $NF}')")
k_cone=$(to_us "$(echo "$clar_out" | grep "scale cones" | awk '{print $NF}')")
k_setup_ms=$(echo "$clar_out" | grep "^Setup:" | awk '{print $2}')
k_setup=$(echo "$k_setup_ms" | awk '{printf "%.0f", $1 * 1000}')
k_status=$(echo "$clar_out" | grep "Terminated" | sed 's/.*status = //')
k_iter=$(echo "$clar_out" | grep "^ *[0-9]" | tail -1 | awk '{print $1}')
k_obj=$(echo "$clar_out" | grep "^ *[0-9]" | tail -1 | awk '{print $2}')

# Per-call averages
c_fac_avg=$(div "$c_fac" "$c_fac_n")
c_solve_avg=$(div "$c_solve" "$c_solve_n")
c_cone_avg=$(div "$c_cone" "$c_iter")
c_select_avg=$(div "${c_select:-0}" "$c_iter")
k_fac_avg=$(div "$k_fac" "$k_iter")
k_solve_avg=$(div "$k_solve" "$k_iter")
k_cone_avg=$(div "$k_cone" "$k_iter")

printf "\n"
printf "  %-28s  %14s  %14s\n" "$NAME" "Conex ($ALG)" "Clarabel"
printf "  %-28s  %14s  %14s\n" "$(printf '%.0s-' {1..28})" "--------------" "--------------"
printf "  %-28s  %11s us  %11s us\n" "Setup/Build" "$c_build" "$k_setup"
printf "  %-28s  %11s us  %11s us\n" "Total solve (sum)" "$c_total" "$k_total"
printf "  %-28s  %14s  %14s\n"       "Iterations" "$c_iter" "$k_iter"
printf "  %-28s  %14s  %14s\n"       "Factorizations" "$c_nfac" "$k_iter"
printf "\n"
printf "  %-28s  %11s us  %11s us\n" "Factor time (sum)" "$c_fac" "$k_fac"
printf "  %-28s  %11s us  %11s us\n" "  per factorization (avg)" "$c_fac_avg" "$k_fac_avg"
printf "  %-28s  %11s us  %11s us\n" "Back-solve time (sum)" "$c_solve" "$k_solve"
printf "  %-28s  %11s us  %11s us\n" "  per solve (avg)" "$c_solve_avg" "$k_solve_avg"
printf "  %-28s  %11s us  %11s us\n" "Cone ops (sum)" "$c_cone" "$k_cone"
printf "  %-28s  %11s us  %11s us\n" "  per iteration (avg)" "$c_cone_avg" "$k_cone_avg"
printf "  %-28s  %11s us  %14s\n"    "Selection/other (sum)" "${c_select:-0}" "—"
printf "  %-28s  %11s us  %14s\n"    "  per iteration (avg)" "${c_select_avg:-0}" "—"
printf "\n"
printf "  %-28s  %14s  %14s\n"       "Objective" "$c_obj" "$k_obj"
printf "  %-28s  %14s  %14s\n"       "Status/Gap" "$c_gap" "$k_status"
