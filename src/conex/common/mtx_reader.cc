#include "conex/common/mtx_reader.h"

#include <cstdlib>
#include <fstream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>

namespace conex {

std::pair<Model, MTXInfo> ReadMTX(const std::string& path,
                                    bool randomize) {
  std::ifstream f(path);
  if (!f.is_open()) {
    throw std::runtime_error("Cannot open: " + path);
  }

  // Parse header comments.
  std::string line;
  bool symmetric = false;
  bool pattern_only = false;
  while (std::getline(f, line)) {
    if (line.find("symmetric") != std::string::npos) symmetric = true;
    if (line.find("pattern") != std::string::npos) pattern_only = true;
    if (line[0] != '%') break;
  }

  int nrows, ncols, nnz;
  std::istringstream(line) >> nrows >> ncols >> nnz;

  // Read triplets.
  std::vector<Eigen::Triplet<double>> triplets;
  for (int i = 0; i < nnz; i++) {
    int r, c;
    double v = 1.0;
    if (pattern_only) {
      f >> r >> c;
    } else {
      f >> r >> c >> v;
    }
    r--; c--;  // 1-based to 0-based
    triplets.emplace_back(r, c, v);
    if (symmetric && r != c) {
      triplets.emplace_back(c, r, v);
    }
  }
  Eigen::SparseMatrix<double> A(nrows, ncols);
  A.setFromTriplets(triplets.begin(), triplets.end());

  // Extract name from path.
  MTXInfo info;
  auto slash = path.rfind('/');
  auto dot = path.rfind('.');
  info.name = path.substr(slash == std::string::npos ? 0 : slash + 1,
                           dot - (slash == std::string::npos ? 0 : slash + 1));
  info.rows = nrows;
  info.cols = ncols;
  info.nnz = A.nonZeros();

  // Randomize values if requested.
  if (randomize) {
    srand(42);
    for (int k = 0; k < A.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
        it.valueRef() = (double)rand() / RAND_MAX - 0.5;
    info.name += "_rand";
  }

  // Classify and build problem.
  info.is_quadratic = (nrows == ncols);
  info.was_transposed = false;

  // Transpose wide matrices to tall.
  if (!info.is_quadratic && A.rows() < A.cols()) {
    A = Eigen::SparseMatrix<double>(A.transpose());
    info.was_transposed = true;
  }

  const int num_vars = A.cols();
  std::vector<int> vars(num_vars);
  std::iota(vars.begin(), vars.end(), 0);

  Model problem;
  if (info.is_quadratic) {
    // Square matrix: treat as Q in min 0.5 x^T Q x.
    // Symmetrize: Q = A + A^T to ensure SPD-like structure.
    Eigen::SparseMatrix<double> Q =
        A + Eigen::SparseMatrix<double>(A.transpose());
    // Add diagonal to ensure positive definiteness.
    for (int i = 0; i < num_vars; ++i)
      Q.coeffRef(i, i) += num_vars;
    problem.AddQuadraticCost(Q, vars);
  } else {
    problem.AddLinearConstraint(A, Eigen::VectorXd::Zero(A.rows()), vars);
  }

  return {problem, info};
}

}  // namespace conex
