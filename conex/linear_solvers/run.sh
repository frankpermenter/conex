#cmake --build build -j$(nproc) && ./build/solve_lp_test 
set -e
cmake --build build --target eval_embedding -j$(nproc)
#./build/solver_comparison 50 20 0 42
#./build/solver_comparison 50 20 0 4 10 10
#/agent-workspace/conex/conex/linear_solvers/build/benchmark_center /agent-workspace/conex/conex/linear_solvers/benchmark_data/buck3.dat-s 10
#/agent-workspace/conex/conex/linear_solvers/build/solver_solve_test --gtest_filter="SolverSolve.QP*"

#./build/eval_embedding --n 5 --m 2 --seeds 1 --eps 1e-12 --verbose --algo default --lu
#./build/compare_embedding --verbose
#./build/compare_embedding --verbose --n 7 -lu
./build/compare_embedding --n 110 --m 100 --lu --verbose

