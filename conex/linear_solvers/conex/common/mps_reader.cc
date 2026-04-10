#include "conex/common/mps_reader.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
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

// Parse fixed-format MPS fields.  Fields are at columns:
// [1:2] indicator, [4:12] field1, [14:22] field2, [24:36] field3,
// [39:47] field4, [49:61] field5.
// We use a more lenient whitespace-based parser for compatibility.
struct MPSFields {
  std::string f1, f2, f3, f4, f5;
};

MPSFields ParseFields(const std::string& line) {
  MPSFields f;
  std::istringstream iss(line);
  iss >> f.f1 >> f.f2 >> f.f3 >> f.f4 >> f.f5;
  return f;
}

}  // namespace

std::pair<Problem, MPSInfo> ReadMPS(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open MPS file: " + filename);
  }

  MPSInfo info;

  // Row name → (type, index).
  enum RowType { N_OBJ, L_LE, G_GE, E_EQ };
  struct RowInfo {
    RowType type;
    int index;  // within its type group
  };
  std::map<std::string, RowInfo> rows;
  std::string obj_row;
  int n_le = 0, n_ge = 0, n_eq = 0;

  // Column name → variable index.
  std::map<std::string, int> cols;
  int n_vars = 0;

  // Triplets for each constraint type.
  std::vector<Eigen::Triplet<double>> le_trips, ge_trips, eq_trips;
  Eigen::VectorXd obj_coeffs;  // allocated after we know n_vars

  // RHS values.
  std::map<std::string, double> rhs_le, rhs_ge, rhs_eq;

  // Bounds.
  struct Bound {
    double lo = 0;
    double up = 1e30;
    bool has_lo = true;
    bool has_up = false;
  };
  std::map<std::string, Bound> bounds;

  // Objective coefficients stored temporarily.
  std::map<int, double> obj_map;

  enum Section { NONE, NAME, ROWS, COLUMNS, RHS, RANGES, BOUNDS, ENDATA };
  Section section = NONE;

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '*') continue;
    std::string trimmed = Trim(line);
    if (trimmed.empty()) continue;

    // Section headers start in column 0 (no leading space).
    if (line[0] != ' ' && line[0] != '\t') {
      if (trimmed == "NAME" || trimmed.substr(0, 4) == "NAME") {
        section = NAME;
        if (trimmed.size() > 4) info.name = Trim(trimmed.substr(4));
        continue;
      }
      if (trimmed == "ROWS") { section = ROWS; continue; }
      if (trimmed == "COLUMNS") { section = COLUMNS; continue; }
      if (trimmed == "RHS") { section = RHS; continue; }
      if (trimmed == "RANGES") { section = RANGES; continue; }
      if (trimmed == "BOUNDS") { section = BOUNDS; continue; }
      if (trimmed == "ENDATA") { section = ENDATA; break; }
      continue;
    }

    auto f = ParseFields(line);

    switch (section) {
      case ROWS: {
        if (f.f1 == "N") {
          obj_row = f.f2;
          rows[f.f2] = {N_OBJ, 0};
        } else if (f.f1 == "L") {
          rows[f.f2] = {L_LE, n_le++};
        } else if (f.f1 == "G") {
          rows[f.f2] = {G_GE, n_ge++};
        } else if (f.f1 == "E") {
          rows[f.f2] = {E_EQ, n_eq++};
        }
        break;
      }

      case COLUMNS: {
        // f1 = col_name, f2 = row_name, f3 = value [, f4 = row_name, f5 = value]
        std::string col_name = f.f1;
        if (cols.find(col_name) == cols.end()) {
          cols[col_name] = n_vars++;
        }
        int j = cols[col_name];

        auto add_entry = [&](const std::string& row, const std::string& val_str) {
          if (row.empty() || val_str.empty()) return;
          double val = std::stod(val_str);
          if (val == 0) return;
          auto it = rows.find(row);
          if (it == rows.end()) return;
          switch (it->second.type) {
            case N_OBJ: obj_map[j] = val; break;
            case L_LE: le_trips.emplace_back(it->second.index, j, val); break;
            case G_GE: ge_trips.emplace_back(it->second.index, j, val); break;
            case E_EQ: eq_trips.emplace_back(it->second.index, j, val); break;
          }
        };

        add_entry(f.f2, f.f3);
        add_entry(f.f4, f.f5);
        break;
      }

      case RHS: {
        // f1 = rhs_name (ignored), f2 = row_name, f3 = value [, f4, f5]
        auto set_rhs = [&](const std::string& row, const std::string& val_str) {
          if (row.empty() || val_str.empty()) return;
          double val = std::stod(val_str);
          auto it = rows.find(row);
          if (it == rows.end()) return;
          switch (it->second.type) {
            case L_LE: rhs_le[row] = val; break;
            case G_GE: rhs_ge[row] = val; break;
            case E_EQ: rhs_eq[row] = val; break;
            default: break;
          }
        };
        set_rhs(f.f2, f.f3);
        set_rhs(f.f4, f.f5);
        break;
      }

      case BOUNDS: {
        // f1 = bound_type, f2 = bound_name, f3 = col_name, f4 = value
        std::string btype = f.f1;
        std::string col_name = f.f3;
        if (cols.find(col_name) == cols.end()) {
          cols[col_name] = n_vars++;
        }
        auto& b = bounds[col_name];
        if (btype == "LO") {
          b.lo = std::stod(f.f4);
          b.has_lo = true;
        } else if (btype == "UP") {
          b.up = std::stod(f.f4);
          b.has_up = true;
        } else if (btype == "FX") {
          b.lo = b.up = std::stod(f.f4);
          b.has_lo = b.has_up = true;
        } else if (btype == "FR") {
          b.has_lo = false;
          b.has_up = false;
        } else if (btype == "MI") {
          b.lo = -1e30;
          b.has_lo = false;
        } else if (btype == "BV") {
          b.lo = 0; b.up = 1;
          b.has_lo = b.has_up = true;
        }
        info.has_bounds = true;
        break;
      }

      default:
        break;
    }
  }

  // Build the Problem.
  Problem problem;
  std::vector<int> all_vars(n_vars);
  std::iota(all_vars.begin(), all_vars.end(), 0);

  // Objective.
  Eigen::VectorXd c = Eigen::VectorXd::Zero(n_vars);
  for (auto& [j, val] : obj_map) c(j) = val;
  problem.SetLinearCost(c);

  // LE constraints: Ax <= b  →  -Ax + b >= 0  →  AddLinearConstraint(-A, b).
  if (n_le > 0) {
    Eigen::SparseMatrix<double> A_le(n_le, n_vars);
    A_le.setFromTriplets(le_trips.begin(), le_trips.end());
    Eigen::VectorXd b_le = Eigen::VectorXd::Zero(n_le);
    for (auto& [row_name, ri] : rows) {
      if (ri.type == L_LE) {
        auto it = rhs_le.find(row_name);
        if (it != rhs_le.end()) b_le(ri.index) = it->second;
      }
    }
    // Convention: Ax + b >= 0.  LE: Ax <= rhs → -Ax + rhs >= 0.
    Eigen::SparseMatrix<double> neg_A = -A_le;
    problem.AddLinearConstraint(neg_A, b_le, all_vars);
  }

  // GE constraints: Ax >= b  →  Ax - b >= 0  →  AddLinearConstraint(A, -b).
  if (n_ge > 0) {
    Eigen::SparseMatrix<double> A_ge(n_ge, n_vars);
    A_ge.setFromTriplets(ge_trips.begin(), ge_trips.end());
    Eigen::VectorXd b_ge = Eigen::VectorXd::Zero(n_ge);
    for (auto& [row_name, ri] : rows) {
      if (ri.type == G_GE) {
        auto it = rhs_ge.find(row_name);
        if (it != rhs_ge.end()) b_ge(ri.index) = it->second;
      }
    }
    Eigen::VectorXd neg_b = -b_ge;
    problem.AddLinearConstraint(A_ge, neg_b, all_vars);
  }

  // Equality constraints.
  if (n_eq > 0) {
    Eigen::SparseMatrix<double> C(n_eq, n_vars);
    C.setFromTriplets(eq_trips.begin(), eq_trips.end());
    Eigen::VectorXd d = Eigen::VectorXd::Zero(n_eq);
    for (auto& [row_name, ri] : rows) {
      if (ri.type == E_EQ) {
        auto it = rhs_eq.find(row_name);
        if (it != rhs_eq.end()) d(ri.index) = it->second;
      }
    }
    problem.AddEqualityConstraint(C, d, all_vars);
  }

  // Variable bounds as additional inequality constraints.
  // lo <= x_j  →  x_j - lo >= 0  →  e_j x + (-lo) >= 0.
  // x_j <= up  →  -x_j + up >= 0  →  -e_j x + up >= 0.
  {
    std::vector<Eigen::Triplet<double>> lo_trips, up_trips;
    std::vector<double> lo_rhs, up_rhs;
    int n_lo = 0, n_up = 0;

    // Default bounds: 0 <= x_j (no upper bound) unless overridden.
    for (int j = 0; j < n_vars; ++j) {
      double lo_val = 0, up_val = 1e30;
      bool has_lo = true, has_up = false;

      // Find column name for variable j.
      // (Reverse lookup — not ideal but works for moderate sizes.)
      for (auto& [name, idx] : cols) {
        if (idx == j) {
          auto bit = bounds.find(name);
          if (bit != bounds.end()) {
            lo_val = bit->second.lo;
            up_val = bit->second.up;
            has_lo = bit->second.has_lo;
            has_up = bit->second.has_up;
          }
          break;
        }
      }

      if (has_lo && lo_val > -1e20) {
        lo_trips.emplace_back(n_lo, j, 1.0);
        lo_rhs.push_back(-lo_val);
        n_lo++;
      }
      if (has_up && up_val < 1e20) {
        up_trips.emplace_back(n_up, j, -1.0);
        up_rhs.push_back(up_val);
        n_up++;
      }
    }

    if (n_lo > 0) {
      Eigen::SparseMatrix<double> A_lo(n_lo, n_vars);
      A_lo.setFromTriplets(lo_trips.begin(), lo_trips.end());
      Eigen::VectorXd b_lo =
          Eigen::Map<Eigen::VectorXd>(lo_rhs.data(), n_lo);
      problem.AddLinearConstraint(A_lo, b_lo, all_vars);
    }
    if (n_up > 0) {
      Eigen::SparseMatrix<double> A_up(n_up, n_vars);
      A_up.setFromTriplets(up_trips.begin(), up_trips.end());
      Eigen::VectorXd b_up =
          Eigen::Map<Eigen::VectorXd>(up_rhs.data(), n_up);
      problem.AddLinearConstraint(A_up, b_up, all_vars);
    }
  }

  info.num_variables = n_vars;
  info.num_le_rows = n_le;
  info.num_ge_rows = n_ge;
  info.num_eq_rows = n_eq;

  return {std::move(problem), info};
}

}  // namespace conex
