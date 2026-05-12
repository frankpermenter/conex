cmake_minimum_required(VERSION 3.14)
project(conex CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Eigen
find_package(Eigen3 REQUIRED)

# GoogleTest: prefer system install, fall back to FetchContent.
find_package(GTest QUIET)
if(NOT GTest_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
  )
  FetchContent_MakeAvailable(googletest)
endif()

# Use src/ as include root so that #include "conex/common/foo.h" works.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
# Tests directory for shared test headers (kkt_residuals.h).
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tests)

# --- Tree solver core ---
add_library(tree_solver
  src/conex/linear_solvers/kkt_tree_solver.cc
  src/conex/linear_solvers/kkt_subsystem.cc
  src/conex/linear_solvers/assembler_adapter.cc
  src/conex/linear_solvers/tree_utils.cc
)
find_package(BLAS)
target_link_libraries(tree_solver PUBLIC Eigen3::Eigen)
if(BLAS_FOUND)
  target_link_libraries(tree_solver PUBLIC BLAS::BLAS)
  target_compile_definitions(tree_solver PUBLIC
    EIGEN_USE_BLAS EIGEN_USE_LAPACKE
    # Pre-define lapack_complex types as C++ std::complex before Eigen's
    # bundled lapacke.h can define them as C99 _Complex (which breaks gtest).
    "lapack_complex_float=std::complex<float>"
    "lapack_complex_double=std::complex<double>"
  )
  # Ensure <complex> is included before the macros are evaluated.
  target_compile_options(tree_solver PUBLIC -include complex)
  target_link_libraries(tree_solver PUBLIC lapacke)
endif()

# --- Clique ordering ---
add_library(clique_ordering
  src/conex/common/clique_ordering_md.cc
  src/conex/common/pq_tree.cc
)
target_link_libraries(clique_ordering PUBLIC Eigen3::Eigen)

# --- Structural rank ---
add_library(structural_rank
  src/conex/common/structural_rank.cc
)
target_link_libraries(structural_rank PUBLIC Eigen3::Eigen)

# --- Constraint infrastructure ---
add_library(constraints
  src/conex/common/block_partition.cc
  src/conex/common/kkt_solver_interface.cc
  src/conex/common/linear_constraint.cc
  src/conex/common/equality_constraint.cc
  src/conex/common/constraint_manager.cc
  src/conex/common/blas_wrapper.cc
  src/conex/common/psd_cone_ops.cc
  src/conex/common/soc_cone_ops.cc
  src/conex/common/exp_cone_ops.cc
  src/conex/common/power_cone_ops.cc
  src/conex/common/rel_entropy_cone_ops.cc
  src/conex/common/hypo_geomean_cone_ops.cc
  src/conex/common/barrier_cone_operations.cc
  src/conex/common/barrier_integrators.cc
  src/conex/common/soc_linear_constraint.cc
)
target_link_libraries(constraints PUBLIC Eigen3::Eigen clique_ordering)
if(BLAS_FOUND)
  target_link_libraries(constraints PUBLIC BLAS::BLAS)
  target_compile_definitions(constraints PRIVATE EIGEN_USE_BLAS)
endif()

# --- Solver factory (MakeTreeSolver only) ---
add_library(kkt_solver_factory
  src/conex/linear_solvers/kkt_solver_factory.cc
)
target_link_libraries(kkt_solver_factory PUBLIC tree_solver clique_ordering constraints)

# --- Sparse linear constraint ---
add_library(sparse_linear_constraint
  src/conex/common/rescale.cc
  src/conex/common/mps_reader.cc
  src/conex/common/mtx_reader.cc
  src/conex/common/sdpa_reader.cc
  src/conex/common/qps_reader.cc
  src/conex/common/cbf_reader.cc
  src/conex/common/sparse_linear_constraint.cc
  src/conex/common/psd_constraint.cc
  src/conex/common/sparse_psd_constraint.cc
  src/conex/common/sparse_quadratic_term.cc
  src/conex/common/sparse_equality_constraint.cc
  src/conex/common/extended_embedding.cc
  src/conex/common/model.cc
  src/conex/common/chordal_decomp.cc
)
target_link_libraries(sparse_linear_constraint PUBLIC constraints kkt_solver_factory structural_rank)

# --- Algorithms (built on top of solver, no downward dependency) ---
add_library(algorithms
  src/conex/common/affine_projection.cc
  src/conex/algorithms/irls.cc
  src/conex/algorithms/barrier_qp.cc
  src/conex/algorithms/geodesic_ipm.cc
  src/conex/algorithms/geodesic_hybrid_r.cc
  src/conex/algorithms/geodesic_hsde.cc
  src/conex/algorithms/alternating_projections.cc
  src/conex/algorithms/solve_lp.cc
  src/conex/algorithms/least_squares.cc
  src/conex/algorithms/equality_constrained_least_squares.cc
  src/conex/algorithms/finite_horizon.cc
  src/conex/algorithms/lqr_tree_solver.cc
  src/conex/algorithms/tree_solver_builder.cc
  src/conex/algorithms/self_dual_embedding.cc
  src/conex/common/kkt_system.cc
  src/conex/common/solver.cc
)
target_link_libraries(algorithms PUBLIC sparse_linear_constraint)

# --- Test ---
enable_testing()
add_executable(sparse_linear_constraint_test
  tests/sparse_linear_constraint_test.cc
)
target_link_libraries(sparse_linear_constraint_test
  algorithms
  sparse_linear_constraint
  clique_ordering
  GTest::gtest_main
)
add_executable(multithreading_test
  tests/multithreading_test.cc
)
target_link_libraries(multithreading_test
  sparse_linear_constraint
  clique_ordering
  GTest::gtest_main
)
add_executable(lqr_benchmark
  bench/lqr_benchmark.cc
)
target_link_libraries(lqr_benchmark
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(lqr_benchmark PRIVATE -O2)

add_executable(solver_comparison
  bench/solver_comparison.cc
)
target_link_libraries(solver_comparison
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(solver_comparison PRIVATE -O2)

add_executable(multi_rhs_benchmark
  bench/multi_rhs_benchmark.cc
)
target_link_libraries(multi_rhs_benchmark
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(multi_rhs_benchmark PRIVATE -O2)

add_executable(benchmark
  bench/benchmark.cc
)
target_link_libraries(benchmark
  sparse_linear_constraint
  clique_ordering
)
target_compile_options(benchmark PRIVATE -O2 -fno-omit-frame-pointer -DEIGEN_RUNTIME_NO_MALLOC)
target_link_libraries(benchmark openblas)

add_executable(profile_mtx
  bench/profile_mtx.cc
)
target_link_libraries(profile_mtx
  algorithms
  sparse_linear_constraint
  clique_ordering
  kkt_solver_factory
  tree_solver
)
target_compile_options(profile_mtx PRIVATE -O2 -fno-omit-frame-pointer)

add_executable(low_rank_diagonal_test
  tests/low_rank_diagonal_test.cc
)
target_link_libraries(low_rank_diagonal_test
  sparse_linear_constraint
  clique_ordering
  kkt_solver_factory
  tree_solver
  GTest::gtest_main
)

add_executable(sparse_quadratic_term_test
  tests/sparse_quadratic_term_test.cc
)
target_link_libraries(sparse_quadratic_term_test
  algorithms
  sparse_linear_constraint
  clique_ordering
  kkt_solver_factory
  tree_solver
  GTest::gtest_main
)

# --- Optional: GPU tree solver (requires CUDA) ---
# Prefer CUDA 12 if available (needed for cuDSS, fixes nvcc 11.x + GCC 11 issues).
find_program(CUDA12_NVCC nvcc PATHS /usr/local/cuda-12.2/bin /usr/local/cuda-12/bin NO_DEFAULT_PATH)
if(CUDA12_NVCC)
  set(CMAKE_CUDA_COMPILER ${CUDA12_NVCC} CACHE FILEPATH "CUDA compiler" FORCE)
endif()

include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  set(CMAKE_CUDA_STANDARD 17)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "")
    set(CMAKE_CUDA_ARCHITECTURES "60;70;80;89" CACHE STRING "CUDA architectures" FORCE)
  endif()

  add_library(gpu_tree_solver
    src/conex/gpu_tree_solver/gpu_tree_solver.cc
    src/conex/gpu_tree_solver/gpu_device_arena.cc
    src/conex/gpu_tree_solver/gpu_extend_add.cu
  )
  target_link_libraries(gpu_tree_solver PUBLIC
    tree_solver
    Eigen3::Eigen
    CUDA::cusolver
    CUDA::cublas
    CUDA::cudart
  )
  target_include_directories(gpu_tree_solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  add_executable(gpu_tree_solver_test
    tests/gpu_tree_solver_test.cc
  )
  target_link_libraries(gpu_tree_solver_test
    gpu_tree_solver
    sparse_linear_constraint
    clique_ordering
    kkt_solver_factory
    tree_solver
    GTest::gtest_main
  )
  gtest_discover_tests(gpu_tree_solver_test)

  # GPU benchmark (optional, requires cuDSS).
  find_library(CUDSS_LIB cudss HINTS /usr/lib/x86_64-linux-gnu/libcudss/12)
  if(CUDSS_LIB)
    add_executable(gpu_benchmark_cudss
      bench/gpu_benchmark_cudss.cc
    )
    target_include_directories(gpu_benchmark_cudss PRIVATE
      /usr/include/libcudss/12
    )
    target_link_libraries(gpu_benchmark_cudss
      gpu_tree_solver
      sparse_linear_constraint
      clique_ordering
      kkt_solver_factory
      tree_solver
      ${CUDSS_LIB}
    )
    message(STATUS "cuDSS found — building gpu_benchmark_cudss")
  else()
    message(STATUS "cuDSS not found — skipping gpu_benchmark_cudss")
  endif()

  message(STATUS "CUDA found — building gpu_tree_solver + tests")
else()
  message(STATUS "CUDA not found — skipping gpu_tree_solver")
endif()

include(GoogleTest)
gtest_discover_tests(sparse_linear_constraint_test)
gtest_discover_tests(multithreading_test)
gtest_discover_tests(low_rank_diagonal_test)
gtest_discover_tests(sparse_quadratic_term_test)


add_executable(geodesic_ipm_test
  tests/geodesic_ipm_test.cc
)
target_link_libraries(geodesic_ipm_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(geodesic_ipm_test)

add_executable(geodesic_hybrid_r_test
  tests/geodesic_hybrid_r_test.cc
)
target_link_libraries(geodesic_hybrid_r_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(geodesic_hybrid_r_test)

add_executable(exp_cone_test
  tests/exp_cone_test.cc
)
target_link_libraries(exp_cone_test PRIVATE
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(exp_cone_test)

add_executable(polyhedral_cone_test
  tests/polyhedral_cone_test.cc
)
target_link_libraries(polyhedral_cone_test PRIVATE
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(polyhedral_cone_test)

# Derivative computation library (Eigen AutoDiff lift/extract, compiled once).
add_library(derivatives STATIC tests/derivatives.cc)
target_link_libraries(derivatives PUBLIC Eigen3::Eigen)

add_executable(autodiff_barrier_test tests/autodiff_barrier_test.cc)
target_link_libraries(autodiff_barrier_test PRIVATE
  derivatives
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(autodiff_barrier_test)

add_executable(power_cone_ops_test tests/power_cone_ops_test.cc)
target_link_libraries(power_cone_ops_test PRIVATE
  derivatives
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(power_cone_ops_test)

add_executable(rel_entropy_cone_ops_test tests/rel_entropy_cone_ops_test.cc)
target_link_libraries(rel_entropy_cone_ops_test PRIVATE
  derivatives
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(rel_entropy_cone_ops_test)

add_executable(hypo_geomean_cone_ops_test tests/hypo_geomean_cone_ops_test.cc)
target_link_libraries(hypo_geomean_cone_ops_test PRIVATE
  derivatives
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(hypo_geomean_cone_ops_test)

add_executable(barrier_integrators_test tests/barrier_integrators_test.cc)
target_link_libraries(barrier_integrators_test PRIVATE
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(barrier_integrators_test)

add_executable(integrator_evaluation bench/integrator_evaluation.cc)
target_link_libraries(integrator_evaluation PRIVATE
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(extended_embedding_test
  tests/extended_embedding_test.cc
)
target_link_libraries(extended_embedding_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(extended_embedding_test)

add_executable(alternating_projections_test
  tests/alternating_projections_test.cc
)
target_link_libraries(alternating_projections_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)

add_executable(psd_ap_test
  tests/psd_ap_test.cc
)
target_link_libraries(psd_ap_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(psd_ap_test)

add_executable(psd_multi_clique_gram_test
  tests/psd_multi_clique_gram_test.cc
)
target_link_libraries(psd_multi_clique_gram_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(psd_multi_clique_gram_test)

add_executable(hybrid_transition_test
  tests/hybrid_transition_test.cc
)
target_link_libraries(hybrid_transition_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(hybrid_transition_test)

add_executable(soc_gram_test
  tests/soc_gram_test.cc
)
target_link_libraries(soc_gram_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(soc_gram_test)

add_executable(soc_decomp_test
  tests/soc_decomp_test.cc
)
target_link_libraries(soc_decomp_test
  GTest::gtest_main
  Eigen3::Eigen
)
gtest_discover_tests(soc_decomp_test)

add_executable(soc_cone_ops_test
  tests/soc_cone_ops_test.cc
)
target_link_libraries(soc_cone_ops_test
  constraints
  GTest::gtest_main
)
gtest_discover_tests(soc_cone_ops_test)

add_executable(benchmark_solver
  bench/benchmark_solver.cc
)
target_link_libraries(benchmark_solver
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(benchmark_solver PRIVATE -O2)

add_executable(benchmark_center
  bench/benchmark_center.cc
)
target_link_libraries(benchmark_center
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(benchmark_center PRIVATE -O2)

add_executable(gram_solve_test
  tests/gram_solve_test.cc
)
target_link_libraries(gram_solve_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(gram_solve_test PRIVATE -O2)

add_executable(synthetic_gram_test
  tests/synthetic_gram_test.cc
)
target_link_libraries(synthetic_gram_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(synthetic_gram_test PRIVATE -O2)
gtest_discover_tests(alternating_projections_test)

add_executable(solve_lp_test
  tests/solve_lp_test.cc
)
target_link_libraries(solve_lp_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(solve_lp_test)

add_executable(problem_solver_test
  tests/problem_solver_test.cc
)
target_link_libraries(problem_solver_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(problem_solver_test)

add_executable(solver_solve_test
  tests/solver_solve_test.cc
)
target_link_libraries(solver_solve_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(solver_solve_test)

add_executable(rescale_test
  tests/rescale_test.cc
)
target_link_libraries(rescale_test
  sparse_linear_constraint
  GTest::gtest_main
)
gtest_discover_tests(rescale_test)

add_executable(self_dual_embedding_test
  tests/self_dual_embedding_test.cc
)
target_link_libraries(self_dual_embedding_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(self_dual_embedding_test)

add_executable(linesearch_bug_test
  tests/linesearch_bug_test.cc
)
target_link_libraries(linesearch_bug_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(linesearch_bug_test)

add_executable(chordal_maxcut_test
  tests/chordal_maxcut_test.cc
)
target_link_libraries(chordal_maxcut_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(equality_ab_test
  tests/equality_ab_test.cc
)
target_link_libraries(equality_ab_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(equality_nontrivial_test
  tests/equality_nontrivial_test.cc
)
target_link_libraries(equality_nontrivial_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(canary bench/canary.cc)
target_link_libraries(canary
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(qps_reader_test tests/qps_reader_test.cc)
target_link_libraries(qps_reader_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(debug_problem bench/debug_problem.cc)
target_link_libraries(debug_problem
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(debug_problem PRIVATE -O2)

add_executable(tune_qps bench/tune_qps.cc)
target_link_libraries(tune_qps
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(tune_qps PRIVATE -O2)

add_executable(tune_hybrid bench/tune_hybrid.cc)
target_link_libraries(tune_hybrid
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(tune_hybrid PRIVATE -O2)

add_executable(eval_embedding bench/eval_embedding.cc)
target_link_libraries(eval_embedding
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(eval_embedding PRIVATE -O2)

add_executable(compare_embedding bench/compare_embedding.cc)
target_link_libraries(compare_embedding
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(compare_embedding PRIVATE -O2)

add_executable(eval_thetacontr bench/eval_thetacontr.cc)
target_link_libraries(eval_thetacontr
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(eval_thetacontr PRIVATE -O2)

add_executable(test_thetacontr bench/test_thetacontr.cc)
target_link_libraries(test_thetacontr
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(test_thetacontr PRIVATE -O2)

add_executable(tree_structure_test tests/tree_structure_test.cc)
target_link_libraries(tree_structure_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)

add_executable(sign_check bench/sign_check.cc)
target_link_libraries(sign_check
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(eq_err_test tests/eq_err_test.cc)
target_link_libraries(eq_err_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(benchmark_qp bench/benchmark_qp.cc)
target_link_libraries(benchmark_qp
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
add_executable(qp_equality_test tests/qp_equality_test.cc)
target_link_libraries(qp_equality_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(kkt_residual_test tests/kkt_residual_test.cc)
target_link_libraries(kkt_residual_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(delayed_ordering_test tests/delayed_ordering_test.cc)
target_link_libraries(delayed_ordering_test clique_ordering)
add_executable(analyze_tree bench/analyze_tree.cc)
target_link_libraries(analyze_tree algorithms constraints sparse_linear_constraint clique_ordering kkt_solver_factory tree_solver structural_rank)

# --- Python bindings (optional) ---
find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 QUIET)
if(NOT pybind11_FOUND AND Python3_FOUND)
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
    OUTPUT_VARIABLE pybind11_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(pybind11_DIR)
    find_package(pybind11 PATHS ${pybind11_DIR} NO_DEFAULT_PATH)
  endif()
endif()

if(pybind11_FOUND)
  pybind11_add_module(_conex python/conex_pybind.cc)
  target_link_libraries(_conex PRIVATE
    algorithms constraints sparse_linear_constraint
    clique_ordering kkt_solver_factory tree_solver
    structural_rank
  )
  message(STATUS "Python bindings enabled")
else()
  message(STATUS "pybind11 not found — skipping Python bindings")
endif()
