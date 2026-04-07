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
  conex/common/linear_constraint.cc
  conex/common/equality_constraint.cc
  conex/common/constraint_manager.cc
  conex/common/blas_wrapper.cc
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
  conex/common/sparse_linear_constraint.cc
  conex/common/sparse_quadratic_term.cc
  conex/common/sparse_equality_constraint.cc
  conex/common/problem.cc
)
target_link_libraries(sparse_linear_constraint PUBLIC constraints kkt_solver_factory structural_rank)

# --- Algorithms (built on top of solver, no downward dependency) ---
add_library(algorithms
  conex/algorithms/irls.cc
  conex/algorithms/barrier_qp.cc
  conex/algorithms/geodesic_barrier_qp.cc
  conex/algorithms/least_squares.cc
  conex/algorithms/equality_constrained_least_squares.cc
  conex/algorithms/finite_horizon.cc
  conex/algorithms/lqr_tree_solver.cc
  conex/algorithms/tree_solver_builder.cc
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

add_executable(dense_kkt_solver_test
  conex/test/dense_kkt_solver_test.cc
)
target_link_libraries(dense_kkt_solver_test
  algorithms tree_solver
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
gtest_discover_tests(dense_kkt_solver_test)
gtest_discover_tests(sparse_linear_constraint_test)
gtest_discover_tests(multithreading_test)
gtest_discover_tests(low_rank_diagonal_test)
gtest_discover_tests(sparse_quadratic_term_test)


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
