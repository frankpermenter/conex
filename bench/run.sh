#cmake --build build -j$(nproc) && ./build/solve_lp_test 
set -e
cmake --build build --target compare_embedding -j$(nproc)
cmake --build build --target benchmark_qp -j$(nproc)
cmake --build build --target benchmark_solver -j$(nproc)
cmake --build build --target geodesic_ipm_test -j$(nproc)
#./build/solver_comparison 50 20 0 42
#./build/solver_comparison 50 20 0 4 10 10
#/agent-workspace/conex/conex/linear_solvers/build/benchmark_center /agent-workspace/conex/conex/linear_solvers/benchmark_data/buck3.dat-s 10
#/agent-workspace/conex/conex/linear_solvers/build/solver_solve_test --gtest_filter="SolverSolve.QP*"

#./build/eval_embedding --n 5 --m 2 --seeds 1 --eps 1e-12 --verbose --algo default --lu
#./build/compare_embedding --verbose
#./build/compare_embedding --verbose --n 7 -lu
#./build/compare_embedding --n 110 --m 100 --lu --verbose

#./build/benchmark_qp /agent-workspace/problem_libraries/maros_meszaros/QPS_Files/QADLITTL.QPS
 #./build/benchmark_qp /agent-workspace/problem_libraries/maros_meszaros/QPS_Files/HS35.QPS --algo ThetaContR
 #./build/eval_embedding --algo ThetaContR --n 3 --m 2 --seeds 4 --verbose
 #./build/eval_embedding --algo ThetaContR --n 20 --seeds 1 --verbose 
# ./build/compare_embedding -n 10 --verbose
 #
 #./build/compare_embedding -n 10 2 --verbose
 #
 #./build/benchmark_qp /agent-workspace/problem_libraries/maros_meszaros/QPS_Files/ZECEVIC2.QPS --verbose
# ./build/compare_embedding -n 10 --verbose

#./build/benchmark_solver /agent-workspace/problem_libraries/SDPLIB/data/qap6.dat-s --algo ThetaContR
#./build/benchmark_solver /agent-workspace/problem_libraries/SDPLIB/data/qap6.data-s --algo ThetaContR
#./build/benchmark_solver /agent-workspace/problem_libraries/SDPLIB/data/qap6.dat-s --algo ThetaContR --tol 1e-9
#./build/benchmark_qp /agent-workspace/problem_libraries/maros_meszaros/QPS_Files/QAFIRO.QPS --algo ThetaContR                                                                                                                                                                    
#./build/compare_embedding -n 10 -v 2>&1 | sed -n '/out.*w_tau.*r_tau/,/^n=/p' | head -30
#./build/compare_embedding -n 10 -v 2
#./build/compare_embedding -n 10 -v  

#./build/test_thetacontr
./build/geodesic_ipm_test --gtest_filter="SpinFactor.*" --gtest_print_time=0
