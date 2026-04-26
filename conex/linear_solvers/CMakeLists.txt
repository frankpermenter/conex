cmake_minimum_required(VERSION 3.14)
project(linear_solvers CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# Use this directory as an include root so that #include "conex/common/foo.h" works.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# --- Tree solver core ---
add_library(tree_solver
  conex/tree_solver/kkt_tree_solver.cc
  conex/tree_solver/kkt_subsystem.cc
  conex/tree_solver/assembler_adapter.cc
  conex/tree_solver/tree_utils.cc
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
  conex/common/clique_ordering_md.cc
  conex/common/pq_tree.cc
)
target_link_libraries(clique_ordering PUBLIC Eigen3::Eigen)

# --- Structural rank ---
add_library(structural_rank
  conex/common/structural_rank.cc
)
target_link_libraries(structural_rank PUBLIC Eigen3::Eigen)

# --- Constraint infrastructure ---
add_library(constraints
  conex/common/block_partition.cc
  conex/common/kkt_solver_interface.cc
  conex/common/linear_constraint.cc
  conex/common/equality_constraint.cc
  conex/common/constraint_manager.cc
  conex/common/blas_wrapper.cc
  conex/common/psd_cone_ops.cc
  conex/common/soc_cone_ops.cc
  conex/common/exp_cone_ops.cc
  conex/common/soc_linear_constraint.cc
)
target_link_libraries(constraints PUBLIC Eigen3::Eigen clique_ordering)
if(BLAS_FOUND)
  target_link_libraries(constraints PUBLIC BLAS::BLAS)
  target_compile_definitions(constraints PRIVATE EIGEN_USE_BLAS)
endif()

# --- Solver factory (MakeTreeSolver only) ---
add_library(kkt_solver_factory
  conex/tree_solver/kkt_solver_factory.cc
)
target_link_libraries(kkt_solver_factory PUBLIC tree_solver clique_ordering constraints)

# --- Sparse linear constraint ---
add_library(sparse_linear_constraint
  conex/common/rescale.cc
  conex/common/mps_reader.cc
  conex/common/mtx_reader.cc
  conex/common/sdpa_reader.cc
  conex/common/qps_reader.cc
  conex/common/cbf_reader.cc
  conex/common/sparse_linear_constraint.cc
  conex/common/psd_constraint.cc
  conex/common/sparse_psd_constraint.cc
  conex/common/sparse_quadratic_term.cc
  conex/common/sparse_equality_constraint.cc
  conex/common/extended_embedding.cc
  conex/common/model.cc
  conex/common/chordal_decomp.cc
)
target_link_libraries(sparse_linear_constraint PUBLIC constraints kkt_solver_factory structural_rank)

# --- Algorithms (built on top of solver, no downward dependency) ---
add_library(algorithms
  conex/common/affine_projection.cc
  conex/algorithms/irls.cc
  conex/algorithms/barrier_qp.cc
  conex/algorithms/geodesic_ipm.cc
  conex/algorithms/geodesic_hybrid_r.cc
  conex/algorithms/alternating_projections.cc
  conex/algorithms/solve_lp.cc
  conex/algorithms/least_squares.cc
  conex/algorithms/equality_constrained_least_squares.cc
  conex/algorithms/finite_horizon.cc
  conex/algorithms/lqr_tree_solver.cc
  conex/algorithms/tree_solver_builder.cc
  conex/algorithms/self_dual_embedding.cc
  conex/common/kkt_system.cc
  conex/common/solver.cc
)
target_link_libraries(algorithms PUBLIC sparse_linear_constraint)

# --- Test ---
enable_testing()
add_executable(sparse_linear_constraint_test
  conex/test/sparse_linear_constraint_test.cc
)
target_link_libraries(sparse_linear_constraint_test
  algorithms
  sparse_linear_constraint
  clique_ordering
  GTest::gtest_main
)
add_executable(multithreading_test
  conex/test/multithreading_test.cc
)
target_link_libraries(multithreading_test
  sparse_linear_constraint
  clique_ordering
  GTest::gtest_main
)
add_executable(lqr_benchmark
  conex/test/lqr_benchmark.cc
)
target_link_libraries(lqr_benchmark
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(lqr_benchmark PRIVATE -O2)

add_executable(solver_comparison
  conex/test/solver_comparison.cc
)
target_link_libraries(solver_comparison
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(solver_comparison PRIVATE -O2)

add_executable(multi_rhs_benchmark
  conex/test/multi_rhs_benchmark.cc
)
target_link_libraries(multi_rhs_benchmark
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(multi_rhs_benchmark PRIVATE -O2)

add_executable(benchmark
  conex/test/benchmark.cc
)
target_link_libraries(benchmark
  sparse_linear_constraint
  clique_ordering
)
target_compile_options(benchmark PRIVATE -O2 -fno-omit-frame-pointer -DEIGEN_RUNTIME_NO_MALLOC)
target_link_libraries(benchmark openblas)

add_executable(profile_mtx
  conex/test/profile_mtx.cc
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
  conex/test/low_rank_diagonal_test.cc
)
target_link_libraries(low_rank_diagonal_test
  sparse_linear_constraint
  clique_ordering
  kkt_solver_factory
  tree_solver
  GTest::gtest_main
)

add_executable(sparse_quadratic_term_test
  conex/test/sparse_quadratic_term_test.cc
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
    conex/gpu_tree_solver/gpu_tree_solver.cc
    conex/gpu_tree_solver/gpu_device_arena.cc
    conex/gpu_tree_solver/gpu_extend_add.cu
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
    conex/test/gpu_tree_solver_test.cc
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
      conex/test/gpu_benchmark_cudss.cc
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
  conex/test/geodesic_ipm_test.cc
)
target_link_libraries(geodesic_ipm_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(geodesic_ipm_test)

add_executable(geodesic_hybrid_r_test
  conex/test/geodesic_hybrid_r_test.cc
)
target_link_libraries(geodesic_hybrid_r_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(geodesic_hybrid_r_test)

add_executable(exp_cone_test
  conex/test/exp_cone_test.cc
)
target_link_libraries(exp_cone_test PRIVATE
  constraints
  GTest::gtest_main
)
gtest_discover_tests(exp_cone_test)

add_executable(extended_embedding_test
  conex/test/extended_embedding_test.cc
)
target_link_libraries(extended_embedding_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(extended_embedding_test)

add_executable(alternating_projections_test
  conex/test/alternating_projections_test.cc
)
target_link_libraries(alternating_projections_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)

add_executable(psd_ap_test
  conex/test/psd_ap_test.cc
)
target_link_libraries(psd_ap_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(psd_ap_test)

add_executable(psd_multi_clique_gram_test
  conex/test/psd_multi_clique_gram_test.cc
)
target_link_libraries(psd_multi_clique_gram_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(psd_multi_clique_gram_test)

add_executable(hybrid_transition_test
  conex/test/hybrid_transition_test.cc
)
target_link_libraries(hybrid_transition_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(hybrid_transition_test)

add_executable(soc_gram_test
  conex/test/soc_gram_test.cc
)
target_link_libraries(soc_gram_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(soc_gram_test)

add_executable(soc_decomp_test
  conex/test/soc_decomp_test.cc
)
target_link_libraries(soc_decomp_test
  GTest::gtest_main
  Eigen3::Eigen
)
gtest_discover_tests(soc_decomp_test)

add_executable(soc_cone_ops_test
  conex/test/soc_cone_ops_test.cc
)
target_link_libraries(soc_cone_ops_test
  constraints
  GTest::gtest_main
)
gtest_discover_tests(soc_cone_ops_test)

add_executable(benchmark_solver
  conex/test/benchmark_solver.cc
)
target_link_libraries(benchmark_solver
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(benchmark_solver PRIVATE -O2)

add_executable(benchmark_center
  conex/test/benchmark_center.cc
)
target_link_libraries(benchmark_center
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(benchmark_center PRIVATE -O2)

add_executable(gram_solve_test
  conex/test/gram_solve_test.cc
)
target_link_libraries(gram_solve_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(gram_solve_test PRIVATE -O2)

add_executable(synthetic_gram_test
  conex/test/synthetic_gram_test.cc
)
target_link_libraries(synthetic_gram_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(synthetic_gram_test PRIVATE -O2)
gtest_discover_tests(alternating_projections_test)

add_executable(solve_lp_test
  conex/test/solve_lp_test.cc
)
target_link_libraries(solve_lp_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(solve_lp_test)

add_executable(problem_solver_test
  conex/test/problem_solver_test.cc
)
target_link_libraries(problem_solver_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(problem_solver_test)

add_executable(solver_solve_test
  conex/test/solver_solve_test.cc
)
target_link_libraries(solver_solve_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(solver_solve_test)

add_executable(rescale_test
  conex/test/rescale_test.cc
)
target_link_libraries(rescale_test
  sparse_linear_constraint
  GTest::gtest_main
)
gtest_discover_tests(rescale_test)

add_executable(self_dual_embedding_test
  conex/test/self_dual_embedding_test.cc
)
target_link_libraries(self_dual_embedding_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(self_dual_embedding_test)

add_executable(linesearch_bug_test
  conex/test/linesearch_bug_test.cc
)
target_link_libraries(linesearch_bug_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)
gtest_discover_tests(linesearch_bug_test)

add_executable(chordal_maxcut_test
  conex/test/chordal_maxcut_test.cc
)
target_link_libraries(chordal_maxcut_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(equality_ab_test
  conex/test/equality_ab_test.cc
)
target_link_libraries(equality_ab_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(equality_nontrivial_test
  conex/test/equality_nontrivial_test.cc
)
target_link_libraries(equality_nontrivial_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(canary conex/test/canary.cc)
target_link_libraries(canary
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(qps_reader_test conex/test/qps_reader_test.cc)
target_link_libraries(qps_reader_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(debug_problem conex/test/debug_problem.cc)
target_link_libraries(debug_problem
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(debug_problem PRIVATE -O2)

add_executable(tune_qps conex/test/tune_qps.cc)
target_link_libraries(tune_qps
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(tune_qps PRIVATE -O2)

add_executable(tune_hybrid conex/test/tune_hybrid.cc)
target_link_libraries(tune_hybrid
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(tune_hybrid PRIVATE -O2)

add_executable(eval_embedding conex/test/eval_embedding.cc)
target_link_libraries(eval_embedding
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(eval_embedding PRIVATE -O2)

add_executable(compare_embedding conex/test/compare_embedding.cc)
target_link_libraries(compare_embedding
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(compare_embedding PRIVATE -O2)

add_executable(eval_thetacontr conex/test/eval_thetacontr.cc)
target_link_libraries(eval_thetacontr
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(eval_thetacontr PRIVATE -O2)

add_executable(test_thetacontr conex/test/test_thetacontr.cc)
target_link_libraries(test_thetacontr
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
target_compile_options(test_thetacontr PRIVATE -O2)

add_executable(tree_structure_test conex/test/tree_structure_test.cc)
target_link_libraries(tree_structure_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
  GTest::gtest_main
)

add_executable(sign_check conex/test/sign_check.cc)
target_link_libraries(sign_check
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(eq_err_test conex/test/eq_err_test.cc)
target_link_libraries(eq_err_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(benchmark_qp conex/test/benchmark_qp.cc)
target_link_libraries(benchmark_qp
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)
add_executable(qp_equality_test conex/test/qp_equality_test.cc)
target_link_libraries(qp_equality_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(kkt_residual_test conex/test/kkt_residual_test.cc)
target_link_libraries(kkt_residual_test
  algorithms constraints sparse_linear_constraint
  clique_ordering kkt_solver_factory tree_solver
  structural_rank
)

add_executable(delayed_ordering_test conex/test/delayed_ordering_test.cc)
target_link_libraries(delayed_ordering_test clique_ordering)
add_executable(analyze_tree conex/test/analyze_tree.cc)
target_link_libraries(analyze_tree algorithms constraints sparse_linear_constraint clique_ordering kkt_solver_factory tree_solver structural_rank)
