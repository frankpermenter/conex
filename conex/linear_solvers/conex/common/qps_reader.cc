#include "conex/common/qps_reader.h"

#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {
namespace {

const double kInf = std::numeric_limits<double>::infinity();

std::string Trim(const std::string& s) {
  auto start = s.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) return "";
  auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(start, end - start + 1);
}

// Fixed-format MPS field extraction (columns are 1-indexed):
//   Field 1: cols  2-3   (indicator/type)
//   Field 2: cols  5-12  (name)
//   Field 3: cols 15-22  (name)
//   Field 4: cols 25-36  (value)
//   Field 5: cols 40-47  (name)
//   Field 6: cols 50-61  (value)
// We use a lenient parser that also handles free-format files.

struct MPSFields {
  std::string f1, f2, f3, f4, f5, f6;
};

MPSFields ParseFields(const std::string& line) {
  // Try free-format: split on whitespace.
  std::istringstream iss(line);
  std::vector<std::string> tokens;
  std::string tok;
  while (iss >> tok) tokens.push_back(tok);

  MPSFields f;
  if (tokens.size() >= 1) f.f1 = tokens[0];
  if (tokens.size() >= 2) f.f2 = tokens[1];
  if (tokens.size() >= 3) f.f3 = tokens[2];
  if (tokens.size() >= 4) f.f4 = tokens[3];
  if (tokens.size() >= 5) f.f5 = tokens[4];
  if (tokens.size() >= 6) f.f6 = tokens[5];
  return f;
}

bool IsSection(const std::string& line) {
  if (line.empty()) return false;
  // Section headers start in column 1 (no leading space).
  return line[0] != ' ' && line[0] != '\t';
}

enum RowType { ROW_N, ROW_E, ROW_L, ROW_G };

}  // namespace

std::pair<Problem, QPSInfo> ReadQPS(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open())
    throw std::runtime_error("Cannot open QPS file: " + filename);

  // --- Pass 1: Parse all raw data ---
  std::string obj_row;
  std::map<std::string, int> row_index;    // row name → index (0-based)
  std::vector<RowType> row_types;

  std::map<std::string, int> col_index;    // column name → index (0-based)
  std::vector<std::string> col_names;

  // Sparse constraint matrix entries: (row, col, value).
  std::vector<Eigen::Triplet<double>> A_trips;
  // Linear cost from the N row.
  std::map<int, double> obj_coeffs;

  // RHS values per row.
  std::map<int, double> rhs_vals;
  double obj_constant = 0;
  // RANGES values per row.
  std::map<int, double> range_vals;

  // Variable bounds: default is [0, +inf).
  struct Bound { double lo = 0; double up = kInf; };
  std::map<int, Bound> bounds;

  // Quadratic objective: upper triangle entries (i, j, value).
  std::vector<Eigen::Triplet<double>> Q_trips;

  auto get_col = [&](const std::string& name) -> int {
    auto it = col_index.find(name);
    if (it != col_index.end()) return it->second;
    int idx = static_cast<int>(col_names.size());
    col_index[name] = idx;
    col_names.push_back(name);
    return idx;
  };

  std::string line;
  std::string section;

  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '*') continue;
    std::string trimmed = Trim(line);
    if (trimmed.empty()) continue;

    if (IsSection(line)) {
      auto fields = ParseFields(trimmed);
      section = fields.f1;
      if (section == "ENDATA") break;
      continue;
    }

    if (section == "ROWS") {
      auto f = ParseFields(trimmed);
      std::string type_str = f.f1;
      std::string name = f.f2;
      if (type_str == "N") {
        obj_row = name;
        // Don't add to row_index — objective is handled separately.
      } else {
        int ri = static_cast<int>(row_types.size());
        row_index[name] = ri;
        if (type_str == "E") row_types.push_back(ROW_E);
        else if (type_str == "L") row_types.push_back(ROW_L);
        else if (type_str == "G") row_types.push_back(ROW_G);
        else throw std::runtime_error("Unknown row type: " + type_str);
      }
    } else if (section == "COLUMNS") {
      auto f = ParseFields(trimmed);
      int ci = get_col(f.f1);
      // Pairs: (row_name, value), possibly two per line.
      auto add_entry = [&](const std::string& rname, const std::string& val) {
        if (rname.empty() || val.empty()) return;
        double v = std::stod(val);
        if (rname == obj_row) {
          obj_coeffs[ci] += v;
        } else {
          auto it = row_index.find(rname);
          if (it != row_index.end())
            A_trips.emplace_back(it->second, ci, v);
        }
      };
      add_entry(f.f2, f.f3);
      add_entry(f.f4, f.f5);
    } else if (section == "RHS") {
      auto f = ParseFields(trimmed);
      // f.f1 is the RHS vector name (ignored), then pairs.
      auto add_rhs = [&](const std::string& rname, const std::string& val) {
        if (rname.empty() || val.empty()) return;
        double v = std::stod(val);
        if (rname == obj_row) { obj_constant -= v; return; }
        auto it = row_index.find(rname);
        if (it != row_index.end()) rhs_vals[it->second] = v;
      };
      add_rhs(f.f2, f.f3);
      add_rhs(f.f4, f.f5);
    } else if (section == "RANGES") {
      auto f = ParseFields(trimmed);
      auto add_range = [&](const std::string& rname, const std::string& val) {
        if (rname.empty() || val.empty()) return;
        double v = std::stod(val);
        auto it = row_index.find(rname);
        if (it != row_index.end()) range_vals[it->second] = v;
      };
      add_range(f.f2, f.f3);
      add_range(f.f4, f.f5);
    } else if (section == "BOUNDS") {
      auto f = ParseFields(trimmed);
      std::string btype = f.f1;
      // f.f2 is the bound set name (ignored).
      std::string cname = f.f3;
      int ci = get_col(cname);
      auto& bd = bounds[ci];
      if (btype == "LO") {
        bd.lo = std::stod(f.f4);
      } else if (btype == "UP") {
        bd.up = std::stod(f.f4);
      } else if (btype == "FX") {
        double v = std::stod(f.f4);
        bd.lo = v;
        bd.up = v;
      } else if (btype == "FR") {
        bd.lo = -kInf;
        bd.up = kInf;
      } else if (btype == "MI") {
        bd.lo = -kInf;
      } else if (btype == "PL") {
        bd.up = kInf;
      } else if (btype == "BV") {
        bd.lo = 0;
        bd.up = 1;
      }
    } else if (section == "QUADOBJ") {
      auto f = ParseFields(trimmed);
      int ci = get_col(f.f1);
      int cj = get_col(f.f2);
      double v = std::stod(f.f3);
      // QPS stores upper triangle of Q where objective = (1/2)x'Qx.
      Q_trips.emplace_back(ci, cj, v);
      if (ci != cj) Q_trips.emplace_back(cj, ci, v);
      // Second pair on same line (rare but possible).
      if (!f.f4.empty() && !f.f5.empty()) {
        int ck = get_col(f.f4);
        double v2 = std::stod(f.f5);
        Q_trips.emplace_back(ci, ck, v2);
        if (ci != ck) Q_trips.emplace_back(ck, ci, v2);
      }
    }
  }

  const int n = static_cast<int>(col_names.size());
  const int m = static_cast<int>(row_types.size());

  // --- Pass 2: Build Problem ---
  Problem prob;
  QPSInfo info;
  info.num_variables = n;
  info.objective_constant = obj_constant;

  // All variables share indices [0, n).
  std::vector<int> all_vars(n);
  std::iota(all_vars.begin(), all_vars.end(), 0);

  // Build sparse A (m x n).
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(A_trips.begin(), A_trips.end());

  // Build RHS vector.
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(m);
  for (auto& [ri, val] : rhs_vals) rhs(ri) = val;

  // Separate rows by type and add constraints.
  // E rows: Ax = b  → AddEqualityConstraint
  // L rows: Ax <= b → b - Ax >= 0  → AddLinearConstraint(-A, b)
  // G rows: Ax >= b → Ax - b >= 0  → AddLinearConstraint(A, -b)
  std::vector<int> eq_rows, le_rows, ge_rows;
  for (int i = 0; i < m; ++i) {
    switch (row_types[i]) {
      case ROW_E: eq_rows.push_back(i); break;
      case ROW_L: le_rows.push_back(i); break;
      case ROW_G: ge_rows.push_back(i); break;
      default: break;
    }
  }

  // Equality constraints.
  if (!eq_rows.empty()) {
    int p = static_cast<int>(eq_rows.size());
    std::vector<Eigen::Triplet<double>> eq_trips;
    Eigen::VectorXd d(p);
    for (int k = 0; k < p; ++k) {
      int ri = eq_rows[k];
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, ri); it; ++it)
        // Note: A is row-major after setFromTriplets with default ColMajor,
        // but we iterate correctly via InnerIterator on the rows.
        ;  // handled below
      d(k) = rhs(ri);
    }
    // Extract submatrix for equality rows.
    // A is ColMajor, so iterate by columns.
    for (int j = 0; j < n; ++j) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, j); it; ++it) {
        // Find if this row is an equality row.
        for (int k = 0; k < p; ++k) {
          if (eq_rows[k] == it.row()) {
            eq_trips.emplace_back(k, j, it.value());
            break;
          }
        }
      }
    }
    // Faster: build a row-index lookup.
    std::map<int, int> eq_row_to_k;
    for (int k = 0; k < p; ++k) eq_row_to_k[eq_rows[k]] = k;

    eq_trips.clear();
    for (int j = 0; j < n; ++j) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, j); it; ++it) {
        auto kit = eq_row_to_k.find(it.row());
        if (kit != eq_row_to_k.end())
          eq_trips.emplace_back(kit->second, j, it.value());
      }
    }
    Eigen::SparseMatrix<double> C(p, n);
    C.setFromTriplets(eq_trips.begin(), eq_trips.end());
    prob.AddEqualityConstraint(C, d, all_vars);
    info.num_equality_rows = p;
  }

  // L rows: b - Ax >= 0  → stored as (-A)x + b >= 0.
  if (!le_rows.empty()) {
    int p = static_cast<int>(le_rows.size());
    std::map<int, int> le_row_to_k;
    for (int k = 0; k < p; ++k) le_row_to_k[le_rows[k]] = k;

    std::vector<Eigen::Triplet<double>> trips;
    Eigen::VectorXd b_le(p);
    for (int k = 0; k < p; ++k) b_le(k) = rhs(le_rows[k]);

    for (int j = 0; j < n; ++j) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, j); it; ++it) {
        auto kit = le_row_to_k.find(it.row());
        if (kit != le_row_to_k.end())
          trips.emplace_back(kit->second, j, -it.value());
      }
    }
    Eigen::SparseMatrix<double> A_le(p, n);
    A_le.setFromTriplets(trips.begin(), trips.end());
    prob.AddLinearConstraint(A_le, b_le, all_vars);
    info.num_inequality_rows += p;
  }

  // G rows: Ax - b >= 0  → stored as Ax + (-b) >= 0.
  if (!ge_rows.empty()) {
    int p = static_cast<int>(ge_rows.size());
    std::map<int, int> ge_row_to_k;
    for (int k = 0; k < p; ++k) ge_row_to_k[ge_rows[k]] = k;

    std::vector<Eigen::Triplet<double>> trips;
    Eigen::VectorXd b_ge(p);
    for (int k = 0; k < p; ++k) b_ge(k) = -rhs(ge_rows[k]);

    for (int j = 0; j < n; ++j) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, j); it; ++it) {
        auto kit = ge_row_to_k.find(it.row());
        if (kit != ge_row_to_k.end())
          trips.emplace_back(kit->second, j, it.value());
      }
    }
    Eigen::SparseMatrix<double> A_ge(p, n);
    A_ge.setFromTriplets(trips.begin(), trips.end());
    prob.AddLinearConstraint(A_ge, b_ge, all_vars);
    info.num_inequality_rows += p;
  }

  // RANGES: for L row with range r, adds  b - r <= Ax <= b (if r > 0).
  //         for G row with range r, adds  b <= Ax <= b + |r|.
  // Implemented as an additional inequality on the other side.
  for (auto& [ri, r] : range_vals) {
    if (std::abs(r) < 1e-30) continue;
    double b_val = rhs.size() > ri ? rhs(ri) : 0.0;

    // Extract this row from A.
    std::vector<Eigen::Triplet<double>> trips;
    for (int j = 0; j < n; ++j) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, j); it; ++it) {
        if (it.row() == ri)
          trips.emplace_back(0, j, it.value());
      }
    }
    Eigen::SparseMatrix<double> A_row(1, n);
    A_row.setFromTriplets(trips.begin(), trips.end());

    if (row_types[ri] == ROW_L) {
      // Original: Ax <= b.  Range adds: Ax >= b - |r|.
      // i.e., Ax - (b - |r|) >= 0.
      Eigen::VectorXd b_range(1);
      b_range(0) = -(b_val - std::abs(r));
      prob.AddLinearConstraint(A_row, b_range, all_vars);
      info.num_inequality_rows++;
    } else if (row_types[ri] == ROW_G) {
      // Original: Ax >= b.  Range adds: Ax <= b + |r|.
      // i.e., (b + |r|) - Ax >= 0.
      Eigen::SparseMatrix<double> neg_A = -A_row;
      Eigen::VectorXd b_range(1);
      b_range(0) = b_val + std::abs(r);
      prob.AddLinearConstraint(neg_A, b_range, all_vars);
      info.num_inequality_rows++;
    }
    // E rows with ranges: treat as L row with the range (rare).
  }

  // Variable bounds.
  // Default: 0 <= x_i < +inf.  Collect all finite bounds.
  // Lower bounds: x_i >= l  →  x_i - l >= 0  →  I_i x + (-l) >= 0.
  // Upper bounds: x_i <= u  →  u - x_i >= 0  →  (-I_i) x + u >= 0.
  {
    // Merge defaults with explicit bounds.
    std::vector<Bound> all_bounds(n, {0.0, kInf});
    for (auto& [ci, bd] : bounds) all_bounds[ci] = bd;

    // Collect finite lower bounds (skip -inf).
    std::vector<Eigen::Triplet<double>> lo_trips;
    std::vector<double> lo_rhs;
    std::vector<Eigen::Triplet<double>> up_trips;
    std::vector<double> up_rhs;

    int lo_count = 0, up_count = 0;
    for (int i = 0; i < n; ++i) {
      if (all_bounds[i].lo > -1e30) {
        lo_trips.emplace_back(lo_count, i, 1.0);
        lo_rhs.push_back(-all_bounds[i].lo);
        lo_count++;
      }
      if (all_bounds[i].up < 1e30) {
        up_trips.emplace_back(up_count, i, -1.0);
        up_rhs.push_back(all_bounds[i].up);
        up_count++;
      }
    }

    if (lo_count > 0) {
      Eigen::SparseMatrix<double> A_lo(lo_count, n);
      A_lo.setFromTriplets(lo_trips.begin(), lo_trips.end());
      Eigen::VectorXd b_lo = Eigen::Map<Eigen::VectorXd>(lo_rhs.data(), lo_count);
      prob.AddLinearConstraint(A_lo, b_lo, all_vars);
    }
    if (up_count > 0) {
      Eigen::SparseMatrix<double> A_up(up_count, n);
      A_up.setFromTriplets(up_trips.begin(), up_trips.end());
      Eigen::VectorXd b_up = Eigen::Map<Eigen::VectorXd>(up_rhs.data(), up_count);
      prob.AddLinearConstraint(A_up, b_up, all_vars);
    }
    info.num_bounded_vars = lo_count + up_count;

    // Fixed variables (lo == up): add equality constraint.
    std::vector<Eigen::Triplet<double>> fx_trips;
    std::vector<double> fx_rhs;
    int fx_count = 0;
    for (int i = 0; i < n; ++i) {
      if (std::abs(all_bounds[i].lo - all_bounds[i].up) < 1e-30 &&
          all_bounds[i].lo > -1e30) {
        fx_trips.emplace_back(fx_count, i, 1.0);
        fx_rhs.push_back(all_bounds[i].lo);
        fx_count++;
      }
    }
    if (fx_count > 0) {
      Eigen::SparseMatrix<double> C_fx(fx_count, n);
      C_fx.setFromTriplets(fx_trips.begin(), fx_trips.end());
      Eigen::VectorXd d_fx = Eigen::Map<Eigen::VectorXd>(fx_rhs.data(), fx_count);
      prob.AddEqualityConstraint(C_fx, d_fx, all_vars);
      info.num_equality_rows += fx_count;
    }
  }

  // Linear cost.
  Eigen::VectorXd c = Eigen::VectorXd::Zero(n);
  for (auto& [ci, val] : obj_coeffs) c(ci) = val;
  prob.SetLinearCost(c);

  // Quadratic cost.
  if (!Q_trips.empty()) {
    Eigen::SparseMatrix<double> Q(n, n);
    Q.setFromTriplets(Q_trips.begin(), Q_trips.end());
    prob.AddQuadraticCost(Q, all_vars);
    info.num_quadratic_entries = static_cast<int>(Q_trips.size()) / 2;
  }

  return {prob, info};
}

}  // namespace conex
