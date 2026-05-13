#!/bin/bash
# Compare conex and Clarabel on SDPLIB instances
SDPLIB="/agent-workspace/problem_libraries/SDPLIB/data"
CONEX="./build/benchmark_solver"
CLARABEL="/tmp/clarabel_build/release/examples/benchmark_sdp"

printf "%-15s  %8s %5s  %8s %5s  %10s %10s\n" \
  "Problem" "Clar_ms" "C_it" "Conex_ms" "X_fac" "Clar_kkt" "Clar_solve"
printf "%s\n" "$(printf '%.0s-' {1..75})"

for prob in truss1 truss2 truss3 truss4 truss5 arch0 arch2 control1 control2; do
  file="$SDPLIB/${prob}.dat-s"
  [ ! -f "$file" ] && continue

  # Clarabel
  clar=$($CLARABEL "$file" 100 2>&1)
  clar_solve=$(echo "$clar" | grep "^Solve:" | awk '{print $2}')
  clar_iter=$(echo "$clar" | grep "^ *[0-9]" | tail -1 | awk '{print $1}')
  clar_kkt_update=$(echo "$clar" | grep "kkt update" | awk '{print $NF}')
  clar_kkt_solve=$(echo "$clar" | grep "kkt solve" | awk '{print $NF}')
  clar_status=$(echo "$clar" | grep "Terminated" | sed 's/.*status = //')

  # Conex
  conex=$($CONEX "$file" --algorithm thetacontr 2>&1)
  conex_line=$(echo "$conex" | grep "ThetaContR " | head -1)
  conex_fac=$(echo "$conex_line" | awk '{print $2}')
  conex_ms=$(echo "$conex_line" | awk '{for(i=1;i<=NF;i++) if($i+0==$i && $i>0.01) last=$i} END{print last}')

  printf "%-15s  %8s %5s  %8s %5s  %10s %10s  %s\n" \
    "$prob" "$clar_solve" "$clar_iter" "$conex_ms" "$conex_fac" \
    "$clar_kkt_update" "$clar_kkt_solve" "$clar_status"
done
