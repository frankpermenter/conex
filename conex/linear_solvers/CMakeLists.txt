cmake_minimum_required(VERSION 3.14)
project(linear_solvers CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Eigen
find_package(Eigen3 REQUIRED)

# GoogleTest
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.14.0
)
FetchContent_MakeAvailable(googletest)

# Use this directory as an include root so that #include "conex/common/foo.h" works.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# --- Tree solver core ---
add_library(tree_solver
  conex/tree_solver/kkt_tree_solver.cc
  conex/tree_solver/kkt_subsystem.cc
  conex/tree_solver/static_subsystem.cc
  conex/tree_solver/tree_utils.cc
)
target_link_libraries(tree_solver PUBLIC Eigen3::Eigen)

# --- Clique ordering ---
add_library(clique_ordering
  conex/common/clique_ordering_md.cc
  conex/common/pq_tree.cc
)
target_link_libraries(clique_ordering PUBLIC tree_solver)

# --- Constraint infrastructure ---
add_library(constraints
  conex/common/linear_constraint.cc
  conex/common/equality_constraint.cc
  conex/common/constraint_manager.cc
)
target_link_libraries(constraints PUBLIC tree_solver clique_ordering)

# --- Solver factory (MakeTreeSolver only) ---
add_library(kkt_solver_factory
  conex/tree_solver/kkt_solver_factory.cc
)
target_link_libraries(kkt_solver_factory PUBLIC tree_solver clique_ordering constraints)

# --- Sparse linear constraint ---
add_library(sparse_linear_constraint
  conex/common/sparse_linear_constraint.cc
)
target_link_libraries(sparse_linear_constraint PUBLIC constraints kkt_solver_factory)

# --- Test ---
enable_testing()
add_executable(sparse_linear_constraint_test
  conex/test/sparse_linear_constraint_test.cc
)
target_link_libraries(sparse_linear_constraint_test
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
  sparse_linear_constraint
  clique_ordering
  kkt_solver_factory
  tree_solver
)
target_compile_options(profile_mtx PRIVATE -O2 -fno-omit-frame-pointer)

include(GoogleTest)
gtest_discover_tests(sparse_linear_constraint_test)
gtest_discover_tests(multithreading_test)
