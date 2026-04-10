#include "conex/common/sdpa_reader.h"

#include <cmath>
#include <fstream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

namespace {

std::string Trim(const std::string& s) {
  auto start = s.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) return "";
  auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(start, end - start + 1);
}

// Skip comment lines (starting with " or *).
bool IsComment(const std::string& line) {
  if (line.empty()) return true;
  char c = line[0];
  return c == '"' || c == '*' || c == '#';
}

// Read next non-comment line.
bool NextLine(std::ifstream& file, std::string& line) {
  while (std::getline(file, line)) {
    if (!IsComment(line) && !Trim(line).empty()) return true;
  }
  return false;
}

Eigen::SparseMatrix<double> ToSparse(const Eigen::MatrixXd& M) {
  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < M.rows(); ++i)
    for (int j = 0; j < M.cols(); ++j)
      if (std::abs(M(i, j)) > 1e-15)
        trips.emplace_back(i, j, M(i, j));
  Eigen::SparseMatrix<double> S(M.rows(), M.cols());
  S.setFromTriplets(trips.begin(), trips.end());
  return S;
}

}  // namespace

std::pair<Problem, SDPAInfo> ReadSDPA(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open())
    throw std::runtime_error("Cannot open SDPA file: " + filename);

  SDPAInfo info;
  std::string line;

  // Line 1: m (number of dual variables / constraints).
  NextLine(file, line);
  info.num_constraints = std::stoi(Trim(line));
  int m = info.num_constraints;

  // Line 2: nblocks.
  NextLine(file, line);
  info.num_blocks = std::stoi(Trim(line));

  // Line 3: block sizes.
  NextLine(file, line);
  {
    // Handle comma, brace, or space-separated formats.
    for (char& c : line) {
      if (c == ',' || c == '{' || c == '}' || c == '(' || c == ')')
        c = ' ';
    }
    std::istringstream iss(line);
    int sz;
    while (iss >> sz) info.block_sizes.push_back(sz);
  }
  if (static_cast<int>(info.block_sizes.size()) != info.num_blocks)
    throw std::runtime_error("Block size count mismatch");

  // Compute block offsets and total dimension.
  // Negative block size means diagonal block.
  std::vector<int> abs_sizes(info.num_blocks);
  std::vector<bool> is_diag(info.num_blocks);
  std::vector<int> offsets(info.num_blocks);
  int total_dim = 0;
  for (int k = 0; k < info.num_blocks; ++k) {
    is_diag[k] = info.block_sizes[k] < 0;
    abs_sizes[k] = std::abs(info.block_sizes[k]);
    offsets[k] = total_dim;
    total_dim += abs_sizes[k];
  }
  info.total_matrix_dim = total_dim;

  // Line 4: b vector (m entries).
  NextLine(file, line);
  for (char& c : line) {
    if (c == ',' || c == '{' || c == '}') c = ' ';
  }
  Eigen::VectorXd b(m);
  {
    std::istringstream iss(line);
    for (int i = 0; i < m; ++i) {
      if (!(iss >> b(i)))
        throw std::runtime_error("Failed to read b vector");
    }
  }

  // Remaining lines: (constraint_idx, block_idx, row, col, value).
  // constraint_idx = 0 → objective C.
  // constraint_idx = 1..m → constraint A_i.
  // Matrices are symmetric: only upper triangle given.

  // Store dense matrices per block per constraint.
  // C_blocks[k] = dense matrix for block k of C.
  // A_blocks[i][k] = dense matrix for block k of constraint i.
  std::vector<Eigen::MatrixXd> C_blocks(info.num_blocks);
  std::vector<std::vector<Eigen::MatrixXd>> A_blocks(m);
  for (int k = 0; k < info.num_blocks; ++k) {
    int n = abs_sizes[k];
    C_blocks[k] = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < m; ++i) {
      A_blocks[i].resize(info.num_blocks);
      A_blocks[i][k] = Eigen::MatrixXd::Zero(n, n);
    }
  }

  while (NextLine(file, line)) {
    for (char& c : line) {
      if (c == ',') c = ' ';
    }
    std::istringstream iss(line);
    int ci, bi, ri, ci2;
    double val;
    if (!(iss >> ci >> bi >> ri >> ci2 >> val)) continue;

    // SDPA uses 1-based indexing.
    bi -= 1;
    ri -= 1;
    ci2 -= 1;

    if (bi < 0 || bi >= info.num_blocks) continue;
    if (ri < 0 || ri >= abs_sizes[bi]) continue;
    if (ci2 < 0 || ci2 >= abs_sizes[bi]) continue;

    if (ci == 0) {
      // Objective C.
      C_blocks[bi](ri, ci2) = val;
      if (ri != ci2) C_blocks[bi](ci2, ri) = val;
    } else if (ci >= 1 && ci <= m) {
      // Constraint A_{ci-1}.
      A_blocks[ci - 1][bi](ri, ci2) = val;
      if (ri != ci2) A_blocks[ci - 1][bi](ci2, ri) = val;
    }
  }

  // Build Problem.
  // Standard dual form: max b^T y s.t. C - Σ y_i A_i ≽ 0.
  // → AddPSDConstraint(A_list = [-A_1, ..., -A_m], B = C, vars).
  // Cost: min -b^T y (since we minimize).
  Problem problem;
  std::vector<int> vars(m);
  std::iota(vars.begin(), vars.end(), 0);

  // One PSD constraint per block.
  for (int k = 0; k < info.num_blocks; ++k) {
    int n = abs_sizes[k];
    if (n == 0) continue;

    if (is_diag[k]) {
      // Diagonal block: treat as nonneg linear constraint.
      // C_diag - Σ y_i A_i_diag >= 0.
      // → A_row_j x + b_j >= 0 where A_row_j = [-A_1(j,j), ..., -A_m(j,j)]
      //   and b_j = C(j,j).
      std::vector<Eigen::Triplet<double>> trips;
      Eigen::VectorXd bv(n);
      for (int j = 0; j < n; ++j) {
        bv(j) = C_blocks[k](j, j);
        for (int i = 0; i < m; ++i) {
          double val = A_blocks[i][k](j, j);
          if (val != 0) trips.emplace_back(j, i, -val);
        }
      }
      Eigen::SparseMatrix<double> A(n, m);
      A.setFromTriplets(trips.begin(), trips.end());
      problem.AddLinearConstraint(A, bv, vars);
    } else {
      // PSD block.
      std::vector<Eigen::SparseMatrix<double>> A_list;
      for (int i = 0; i < m; ++i) {
        A_list.push_back(ToSparse(-A_blocks[i][k]));
      }
      problem.AddPSDConstraint(A_list, ToSparse(C_blocks[k]), vars);
    }
  }

  // Cost: min -b^T y.
  problem.SetLinearCost(-b);

  return {std::move(problem), info};
}

}  // namespace conex
