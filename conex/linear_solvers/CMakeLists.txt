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

# Use this directory as an include root so that #include "conex/foo.h" works.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# --- Tree solver core ---
add_library(tree_solver
  conex/kkt_tree_solver.cc
  conex/kkt_subsystem.cc
  conex/static_subsystem.cc
  conex/tree_utils.cc
)
target_link_libraries(tree_solver PUBLIC Eigen3::Eigen)

# --- Clique ordering ---
add_library(clique_ordering
  conex/clique_ordering_md.cc
  conex/pq_tree.cc
)
target_link_libraries(clique_ordering PUBLIC tree_solver)

# --- Constraint infrastructure ---
add_library(constraints
  conex/linear_constraint.cc
  conex/equality_constraint.cc
  conex/constraint_manager.cc
)
target_link_libraries(constraints PUBLIC tree_solver clique_ordering)

# --- Solver factory (MakeTreeSolver only) ---
add_library(kkt_solver_factory
  conex/kkt_solver_factory.cc
)
target_link_libraries(kkt_solver_factory PUBLIC tree_solver clique_ordering constraints)

# --- Sparse linear constraint ---
add_library(sparse_linear_constraint
  conex/sparse_linear_constraint.cc
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
include(GoogleTest)
gtest_discover_tests(sparse_linear_constraint_test)
