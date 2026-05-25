#./bench/compare.sh /agent-workspace/problem_libraries/SDPLIB/data/arch0.dat-s thetacont #[lp|barrierlp|thetacont|thetacontr]
#./build/benchmark_solver /agent-workspace/problem_libraries/SDPLIB/data/arch2.dat-s --algo Barrier -v
#./build/benchmark_expcone 10 6 42 --algo BarrierTC -v
#./build/benchmark_expcone 10 6 42 --dump | python3 ./bench/compare_expcone.py

#/agent-workspace/conex/build/geodesic_ipm_test
cmake --build build -j2 --target _conex
export PYTHONPATH=/agent-workspace/conex/build:$PYTHONPATH  
python3 python_run.py
