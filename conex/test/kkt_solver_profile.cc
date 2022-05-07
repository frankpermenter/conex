#include "conex/kkt_subsystem.h"
#include "conex/cholesky_solvers.h"

#include <iostream>
#include <map>
#include <tuple>
#include <numeric>

#include "conex/RLDLT.h"
#include "conex/debug_macros.h"
#include "conex/kkt_solver_interface.h"
#include "conex/kkt_tree_solver.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {
#define CONEX_NOOP(x) (void)x;
namespace {
MatrixXd IncrementSubmatrix(const MatrixXd& full_matrix,
                            const MatrixXd& sub_matrix,
                            const std::vector<int>& c) {
  MatrixXd y = full_matrix;
  int i = 0;
  for (auto ci : c) {
    int j = 0;
    for (auto cj : c) {
      y(ci, cj) += sub_matrix(i, j);
      j++;
    }
    i++;
  }
  return y;
}
}  // namespace

using Eigen::MatrixXd;



struct BlockDiagonalMatrixParameters {
  int block_size;
  int num_blocks;
  int overlap;
};

class TestSystem {
 public:
  using Subsystem = StaticSubsystem<true>;
  void Initialize(const BlockDiagonalMatrixParameters& o) {
    int block_size = o.block_size;
    int num_blocks = o.num_blocks;
    int overlap = o.overlap;

    Eigen::MatrixXd Q(block_size, block_size);
    Q.setConstant(1); Q.diagonal().setConstant(block_size);

    parent.resize(num_blocks, -1);
    int offset = 0;
    int size = block_size * num_blocks - (num_blocks - 1) * overlap;
    full_matrix.resize(size, size); full_matrix.setZero();

    for (int i = 0; i < num_blocks; i++) {
      std::vector<int> vars(block_size); std::iota(vars.begin(), vars.end(), offset); 
      full_matrix = IncrementSubmatrix(full_matrix,  Q, vars);
      subsystems_no_ptr.emplace_back(Q, vars);
      offset += Q.rows() - overlap;
      if (overlap > 0 && i < num_blocks - 1) {
        parent.at(i) = i+1;
      }
    }

    for (int i = 0; i < num_blocks; i++) {
      subsystems.push_back(&subsystems_no_ptr.at(i));
    }

    for (auto& s: subsystems) {
      system.AddSubsystem(s);
    }
  }
  
  SymmetricLinearSystemTreeSolver system;
  std::vector<KKTSubsystem*> subsystems;
  std::vector<int> parent;
  std::vector<Subsystem> subsystems_no_ptr;
  Eigen::MatrixXd full_matrix;
};


void DoSparseSolve(const TestSystem& test) {
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower, 
                        Eigen::NaturalOrdering<int>> llt_sparse;
  Eigen::SparseMatrix<double> sparse_matrix = test.full_matrix.sparseView();
  START_TIMER(Sparse)
    llt_sparse.compute(sparse_matrix);
  END_TIMER
}

void DoRecursiveSolve(TestSystem& test) {
  test.system.Assemble();
  START_TIMER(SupernodalRecursive)
    test.system.Factor();
  END_TIMER

  test.system.Assemble();
  START_TIMER(SupernodalRecursiveAgain)
    test.system.Factor();
  END_TIMER
}

void DoDevirtualizedSolve(TestSystem& test) {
  for (auto& s: test.subsystems_no_ptr) {
    s.Assemble(); 
  }
  START_TIMER(SupernodalNoPtr)
    for (auto& s: test.subsystems_no_ptr) {
      s.DoEliminateSupernodeColumns(); 
    }
  END_TIMER
  for (auto& s: test.subsystems_no_ptr) {
    s.Assemble(); 
  }
  START_TIMER(SupernodalNoPtrAgain)
    for (auto& s: test.subsystems_no_ptr) {
      s.DoEliminateSupernodeColumns(); 
    }
  END_TIMER
}

void DoSerialSolve(TestSystem& test) {
  START_TIMER(SupernodalSerial)
    for (auto& s: test.subsystems) {
      s->Factor(); 
    }
  END_TIMER
  for (auto& s: test.subsystems) {
    s->Assemble(); 
  }

  START_TIMER(SupernodalSerialAgain)
    for (auto& s: test.subsystems) {
      s->Factor(); 
    }
  END_TIMER
}

#if 0
GTEST_TEST(KKTSubsystem, BlockDiagonal) {
  TestSystem test;
  test.Initialize(30, 10, 0);
  auto& system = test.system;
  auto& full_matrix = test.full_matrix;
  auto& parent  = test.parent;
  system.Finalize(parent);
  system.Assemble();

  DoSparseSolve(test);
  DoRecursiveSolve(test);
  DoSerialSolve(test);
  DoDevirtualizedSolve(test);
}
#endif

GTEST_TEST(KKTSubsystem, CliqueIntersectionGraphIsPath) {
  TestSystem test;
  BlockDiagonalMatrixParameters p;
  p.block_size = 100;
  p.num_blocks = 10;
  p.overlap = 31;
  test.Initialize(p);
  auto& system = test.system;
  auto& full_matrix = test.full_matrix;
  auto& parent  = test.parent;
  system.Finalize(parent);
  system.Assemble();
  // avoid bug in KKTMatrix
  if (p.overlap > 0) {
    EXPECT_NEAR((system.KKTMatrix() - test.full_matrix).norm(), 0, 1e-14);
  }

  DoSparseSolve(test);
  DoRecursiveSolve(test);
  // DoSerialSolve(test);
  // DoDevirtualizedSolve(test);
}



}  // namespace conex
