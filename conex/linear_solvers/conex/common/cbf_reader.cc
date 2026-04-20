#include "conex/common/cbf_reader.h"

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

bool NextLine(std::ifstream& file, std::string& line) {
  while (std::getline(file, line)) {
    std::string t = Trim(line);
    if (!t.empty() && t[0] != '#') { line = t; return true; }
  }
  return false;
}

// Cone type for scalar (non-PSD) cones.
enum ConeType { FREE, NONNEG, NONPOS, SOC, RSOC };

struct ConeSpec {
  ConeType type;
  int dim;
};

ConeType ParseConeType(const std::string& s) {
  if (s == "F") return FREE;
  if (s == "L+" || s == "L=") return NONNEG;
  if (s == "L-") return NONPOS;
  if (s == "Q") return SOC;
  if (s == "QR") return RSOC;
  throw std::runtime_error("Unknown CBF cone type: " + s);
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

std::pair<Model, CBFInfo> ReadCBF(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open())
    throw std::runtime_error("Cannot open CBF file: " + filename);

  CBFInfo info;
  info.objsense = "MIN";

  int num_vars = 0;
  int num_cons = 0;
  std::vector<ConeSpec> var_cones, con_cones;
  std::vector<int> var_offsets, con_offsets;

  // PSD blocks.
  std::vector<int> psd_var_dims, psd_con_dims;

  // Sparse entries.
  std::vector<Eigen::Triplet<double>> obj_trips;  // objective
  std::vector<Eigen::Triplet<double>> A_trips;     // constraint matrix
  Eigen::VectorXd b_vec;                            // constraint RHS

  // PSD entries: HCOORD (i, j, row, col, val) for PSD constraints,
  // DCOORD (j, row, col, val) for PSD objective.
  struct PSDEntry { int block; int row; int col; double val; };
  std::vector<std::vector<PSDEntry>> psd_con_entries;  // per PSD constraint
  std::vector<PSDEntry> psd_obj_entries;

  std::string line;
  while (NextLine(file, line)) {
    if (line == "VER") {
      NextLine(file, line);  // version number, ignore
    } else if (line == "OBJSENSE") {
      NextLine(file, line);
      info.objsense = line;
    } else if (line == "VAR") {
      // num_vars num_cone_types
      NextLine(file, line);
      std::istringstream iss(line);
      int n_types;
      iss >> num_vars >> n_types;
      int offset = 0;
      for (int i = 0; i < n_types; ++i) {
        NextLine(file, line);
        std::istringstream iss2(line);
        std::string cone_str;
        int dim;
        iss2 >> cone_str >> dim;
        var_cones.push_back({ParseConeType(cone_str), dim});
        var_offsets.push_back(offset);
        offset += dim;
      }
    } else if (line == "CON") {
      NextLine(file, line);
      std::istringstream iss(line);
      int n_types;
      iss >> num_cons >> n_types;
      b_vec = Eigen::VectorXd::Zero(num_cons);
      int offset = 0;
      for (int i = 0; i < n_types; ++i) {
        NextLine(file, line);
        std::istringstream iss2(line);
        std::string cone_str;
        int dim;
        iss2 >> cone_str >> dim;
        con_cones.push_back({ParseConeType(cone_str), dim});
        con_offsets.push_back(offset);
        offset += dim;
      }
    } else if (line == "PSDVAR") {
      NextLine(file, line);
      int n = std::stoi(line);
      info.num_psd_vars = n;
      for (int i = 0; i < n; ++i) {
        NextLine(file, line);
        psd_var_dims.push_back(std::stoi(line));
      }
    } else if (line == "PSDCON") {
      NextLine(file, line);
      int n = std::stoi(line);
      info.num_psd_cons = n;
      psd_con_entries.resize(n);
      for (int i = 0; i < n; ++i) {
        NextLine(file, line);
        psd_con_dims.push_back(std::stoi(line));
      }
    } else if (line == "OBJACOORD") {
      NextLine(file, line);
      int nnz = std::stoi(line);
      for (int i = 0; i < nnz; ++i) {
        NextLine(file, line);
        std::istringstream iss(line);
        int j; double val;
        iss >> j >> val;
        obj_trips.emplace_back(0, j, val);
      }
    } else if (line == "ACOORD") {
      NextLine(file, line);
      int nnz = std::stoi(line);
      for (int k = 0; k < nnz; ++k) {
        NextLine(file, line);
        std::istringstream iss(line);
        int i, j; double val;
        iss >> i >> j >> val;
        A_trips.emplace_back(i, j, val);
      }
    } else if (line == "BCOORD") {
      NextLine(file, line);
      int nnz = std::stoi(line);
      for (int k = 0; k < nnz; ++k) {
        NextLine(file, line);
        std::istringstream iss(line);
        int i; double val;
        iss >> i >> val;
        b_vec(i) = val;
      }
    } else if (line == "HCOORD") {
      NextLine(file, line);
      int nnz = std::stoi(line);
      for (int k = 0; k < nnz; ++k) {
        NextLine(file, line);
        std::istringstream iss(line);
        int ci, vi, r, c; double val;
        iss >> ci >> vi >> r >> c >> val;
        if (ci >= 0 && ci < static_cast<int>(psd_con_entries.size()))
          psd_con_entries[ci].push_back({vi, r, c, val});
      }
    } else if (line == "DCOORD") {
      NextLine(file, line);
      int nnz = std::stoi(line);
      for (int k = 0; k < nnz; ++k) {
        NextLine(file, line);
        std::istringstream iss(line);
        int vi, r, c; double val;
        iss >> vi >> r >> c >> val;
        psd_obj_entries.push_back({vi, r, c, val});
      }
    }
    // Other sections silently ignored.
  }

  info.num_variables = num_vars;
  info.num_constraints = num_cons;

  // Build Model.
  Model problem;
  std::vector<int> all_vars(num_vars);
  std::iota(all_vars.begin(), all_vars.end(), 0);

  // Objective.
  Eigen::VectorXd c = Eigen::VectorXd::Zero(num_vars);
  for (auto& t : obj_trips) c(t.col()) = t.value();
  if (info.objsense == "MAX") c = -c;  // convert to min
  problem.SetLinearCost(c);

  // Build full A matrix.
  Eigen::SparseMatrix<double> A(num_cons, num_vars);
  if (!A_trips.empty()) A.setFromTriplets(A_trips.begin(), A_trips.end());

  // Process constraint cones.
  for (size_t ci = 0; ci < con_cones.size(); ++ci) {
    int off = con_offsets[ci];
    int dim = con_cones[ci].dim;

    // Extract sub-matrix and sub-rhs for this cone.
    // A_sub = A[off:off+dim, :]
    // b_sub = b[off:off+dim]
    std::vector<Eigen::Triplet<double>> sub_trips;
    for (int k = 0; k < A.outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
        if (it.row() >= off && it.row() < off + dim)
          sub_trips.emplace_back(it.row() - off, it.col(), it.value());
      }
    }
    Eigen::SparseMatrix<double> A_sub(dim, num_vars);
    if (!sub_trips.empty())
      A_sub.setFromTriplets(sub_trips.begin(), sub_trips.end());
    Eigen::VectorXd b_sub = b_vec.segment(off, dim);

    switch (con_cones[ci].type) {
      case NONNEG:
        // Ax + b >= 0 (our convention).
        problem.AddLinearConstraint(A_sub, b_sub, all_vars);
        break;
      case NONPOS: {
        // Ax + b <= 0  →  -(Ax + b) >= 0  →  -Ax - b >= 0.
        Eigen::SparseMatrix<double> neg_A = -A_sub;
        Eigen::VectorXd neg_b = -b_sub;
        problem.AddLinearConstraint(neg_A, neg_b, all_vars);
        break;
      }
      case SOC:
        // ||A₁x + b₁|| <= A₀x + b₀.
        // CBF SOC: (A_sub * x + b_sub) in Q.
        problem.AddSOCConstraint(A_sub, b_sub, all_vars);
        break;
      case RSOC:
        // Rotated SOC: 2(Ax+b)_0(Ax+b)_1 >= ||(Ax+b)_{2:}||^2.
        // TODO: convert to standard SOC or add native support.
        throw std::runtime_error("Rotated SOC not yet supported");
      case FREE:
        // Free cone: no constraint. Skip.
        break;
    }
  }

  // TODO: PSD constraints from PSDCON/HCOORD.
  // TODO: Variable cones (var bounds from VAR section).

  return {std::move(problem), info};
}

}  // namespace conex
