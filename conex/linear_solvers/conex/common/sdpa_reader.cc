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

bool IsComment(const std::string& line) {
  if (line.empty()) return true;
  char c = line[0];
  return c == '"' || c == '*' || c == '#';
}

bool NextLine(std::ifstream& file, std::string& line) {
  while (std::getline(file, line)) {
    if (!IsComment(line) && !Trim(line).empty()) return true;
  }
  return false;
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

  // Compute block metadata.
  std::vector<int> abs_sizes(info.num_blocks);
  std::vector<bool> is_diag(info.num_blocks);
  int total_dim = 0;
  for (int k = 0; k < info.num_blocks; ++k) {
    is_diag[k] = info.block_sizes[k] < 0;
    abs_sizes[k] = std::abs(info.block_sizes[k]);
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

  // Read entries directly into sparse triplet lists.
  // C_trips[k] = triplets for block k of objective C.
  // A_trips[i][k] = triplets for block k of constraint A_i.
  std::vector<std::vector<Eigen::Triplet<double>>> C_trips(info.num_blocks);
  std::vector<std::vector<std::vector<Eigen::Triplet<double>>>> A_trips(m);
  for (int i = 0; i < m; ++i) A_trips[i].resize(info.num_blocks);

  while (NextLine(file, line)) {
    for (char& c : line) {
      if (c == ',') c = ' ';
    }
    std::istringstream iss(line);
    int ci, bi, ri, ci2;
    double val;
    if (!(iss >> ci >> bi >> ri >> ci2 >> val)) continue;

    bi -= 1;  // 1-based → 0-based
    ri -= 1;
    ci2 -= 1;

    if (bi < 0 || bi >= info.num_blocks) continue;
    if (ri < 0 || ri >= abs_sizes[bi]) continue;
    if (ci2 < 0 || ci2 >= abs_sizes[bi]) continue;

    if (ci == 0) {
      C_trips[bi].emplace_back(ri, ci2, val);
      if (ri != ci2) C_trips[bi].emplace_back(ci2, ri, val);
    } else if (ci >= 1 && ci <= m) {
      A_trips[ci - 1][bi].emplace_back(ri, ci2, val);
      if (ri != ci2) A_trips[ci - 1][bi].emplace_back(ci2, ri, val);
    }
  }

  // Build Problem.
  // SDPA standard primal: min c^T x s.t. X = Σ x_i F_i - F_0 ≽ 0
  //   (file's `b` vector is SDPA's c; F_0, F_i come from triplets).
  // Conex form Σ A_i x_i + B ≽ 0  ⇒  A_i = F_i, B = -F_0, cost = c.
  Problem problem;
  std::vector<int> vars(m);
  std::iota(vars.begin(), vars.end(), 0);

  for (int k = 0; k < info.num_blocks; ++k) {
    int n = abs_sizes[k];
    if (n == 0) continue;

    // Build F_0 for this block, then negate to get B = -F_0.
    Eigen::SparseMatrix<double> F0_block(n, n);
    F0_block.setFromTriplets(C_trips[k].begin(), C_trips[k].end());
    Eigen::SparseMatrix<double> B_block = -F0_block;

    if (is_diag[k]) {
      // Diagonal block → nonneg constraint Ax + b ≥ 0 with
      // A column i = diag(F_i), b = -diag(F_0).
      std::vector<Eigen::Triplet<double>> trips;
      Eigen::VectorXd bv(n);
      for (int j = 0; j < n; ++j) {
        bv(j) = -F0_block.coeff(j, j);
        for (int i = 0; i < m; ++i) {
          for (const auto& t : A_trips[i][k]) {
            if (t.row() == j && t.col() == j) {
              trips.emplace_back(j, i, t.value());
            }
          }
        }
      }
      Eigen::SparseMatrix<double> A(n, m);
      A.setFromTriplets(trips.begin(), trips.end());
      problem.AddLinearConstraint(A, bv, vars);
    } else {
      // PSD block: A_list[i] = F_i (no negation), B = -F_0.
      std::vector<Eigen::SparseMatrix<double>> A_list;
      for (int i = 0; i < m; ++i) {
        Eigen::SparseMatrix<double> Ai(n, n);
        Ai.setFromTriplets(A_trips[i][k].begin(), A_trips[i][k].end());
        A_list.push_back(std::move(Ai));
      }
      problem.AddPSDConstraint(A_list, B_block, vars,
                               /*use_chordal=*/false);
    }
  }

  problem.SetLinearCost(b);
  return {std::move(problem), info};
}

}  // namespace conex
