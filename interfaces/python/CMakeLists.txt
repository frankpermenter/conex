cmake_minimum_required(VERSION 3.20)
project(conex_python LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(USE_BLAS "Enable BLAS-backed Eigen kernels" OFF)
option(CHECK_EIGEN_MALLOC "Enable Eigen runtime no-malloc checks" OFF)

find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG QUIET)

set(PYBIND11_FALLBACK_INCLUDE "")
if(NOT pybind11_FOUND)
  if(DEFINED ENV{PYBIND11_INCLUDE_DIR} AND EXISTS "$ENV{PYBIND11_INCLUDE_DIR}/pybind11/pybind11.h")
    set(PYBIND11_FALLBACK_INCLUDE "$ENV{PYBIND11_INCLUDE_DIR}")
  elseif(EXISTS "/home/frank/miniconda3/lib/python3.12/site-packages/torch/include/pybind11/pybind11.h")
    set(PYBIND11_FALLBACK_INCLUDE "/home/frank/miniconda3/lib/python3.12/site-packages/torch/include")
  elseif(EXISTS "/usr/include/pybind11/pybind11.h")
    set(PYBIND11_FALLBACK_INCLUDE "/usr/include")
  else()
    message(FATAL_ERROR
      "Could not find pybind11. Install pybind11 or set PYBIND11_INCLUDE_DIR to a directory containing pybind11/pybind11.h.")
  endif()
endif()

find_package(Eigen3 CONFIG QUIET)
if(NOT Eigen3_FOUND)
  if(EXISTS "${CMAKE_SOURCE_DIR}/../../bazel-conex/external/+http_archive+eigen/Eigen")
    set(EIGEN3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../../bazel-conex/external/+http_archive+eigen")
  elseif(DEFINED ENV{EIGEN_DIR} AND EXISTS "$ENV{EIGEN_DIR}/Eigen")
    set(EIGEN3_INCLUDE_DIR "$ENV{EIGEN_DIR}")
  elseif(EXISTS "/usr/include/eigen3/Eigen")
    set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
  else()
    message(FATAL_ERROR "Could not find Eigen. Set EIGEN_DIR to Eigen include root.")
  endif()
endif()

set(CONEX_CORE_SOURCES
  ../conex.cc
  ../conex_qp.cc
  ../../conex/approximate_eigenvalues.cc
  ../../conex/block_triangular_operations.cc
  ../../conex/clique_ordering.cc
  ../../conex/clique_ordering_md.cc
  ../../conex/clique_ordering_utils.cc
  ../../conex/constraint_manager.cc
  ../../conex/conjugate_gradient_solvers.cc
  ../../conex/cone_program.cc
  ../../conex/dense_lmi_constraint.cc
  ../../conex/divergence.cc
  ../../conex/equality_constraint.cc
  ../../conex/exponential_map.cc
  ../../conex/exponential_map_pade.cc
  ../../conex/hermitian_psd.cc
  ../../conex/json_parser.cc
  ../../conex/jordan_matrix_algebra.cc
  ../../conex/kkt_solver.cc
  ../../conex/kkt_solver_factory.cc
  ../../conex/kkt_simplicial_solver.cc
  ../../conex/kkt_subsystem.cc
  ../../conex/kkt_tree_solver.cc
  ../../conex/linear_constraint.cc
  ../../conex/psd_constraint.cc
  ../../conex/quadratic_cone_constraint.cc
  ../../conex/quadratic_cost.cc
  ../../conex/self_dual_embedding.cc
  ../../conex/serialize.cc
  ../../conex/soc_constraint.cc
  ../../conex/static_subsystem.cc
  ../../conex/supernodal_assembler.cc
  ../../conex/supernodal_solver.cc
  ../../conex/tree_utils.cc
  ../../conex/triangular_matrix_workspace.cc
)

if(pybind11_FOUND)
  pybind11_add_module(_conex MODULE
    src/conex_pybind.cpp
    ${CONEX_CORE_SOURCES}
  )
else()
  Python3_add_library(_conex MODULE
    src/conex_pybind.cpp
    ${CONEX_CORE_SOURCES}
  )
  target_link_libraries(_conex PRIVATE Python3::Module)
  target_include_directories(_conex PRIVATE "${PYBIND11_FALLBACK_INCLUDE}")
  target_compile_definitions(_conex PRIVATE PYBIND11_DETAILED_ERROR_MESSAGES=1)
endif()

target_include_directories(_conex PRIVATE
  ..
  ../..
)

if(Eigen3_FOUND)
  target_link_libraries(_conex PRIVATE Eigen3::Eigen)
else()
  target_include_directories(_conex PRIVATE "${EIGEN3_INCLUDE_DIR}")
endif()

# Match historical optimization/visibility/section flags from interfaces/Makefile.
target_compile_options(_conex PRIVATE
  -O3
  -fdata-sections
  -ffunction-sections
  -fvisibility=hidden
)
if(CHECK_EIGEN_MALLOC)
  target_compile_definitions(_conex PRIVATE EIGEN_RUNTIME_NO_MALLOC)
else()
  target_compile_definitions(_conex PRIVATE
    NDEBUG
    EIGEN_NO_DEBUG
  )
endif()
target_link_options(_conex PRIVATE
  -static-libstdc++
  -Wl,--exclude-libs,ALL
  -Wl,--gc-sections
)

find_package(BLAS REQUIRED)
target_compile_definitions(_conex PRIVATE EIGEN_USE_BLAS)
target_link_libraries(_conex PRIVATE BLAS::BLAS)

set_target_properties(_conex PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}"
)
