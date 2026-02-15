#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Sparse>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "../../conex.h"
#include "../../../conex/clique_ordering.h"
#include "../../../conex/kkt_tree_solver.h"
#include "../../../conex/static_subsystem.h"
#include "../../../conex/workspace.h"

namespace py = pybind11;

namespace {

void* PtrFromPy(std::uintptr_t p) { return reinterpret_cast<void*>(p); }
std::uintptr_t PtrToPy(void* p) { return reinterpret_cast<std::uintptr_t>(p); }

class IntPtr {
 public:
  IntPtr(int value = 0) : value_(value) {}
  int value() const { return value_; }
  void set(int v) { value_ = v; }
  int* ptr() { return &value_; }

 private:
  int value_;
};

template <int N>
py::array_t<double, py::array::c_style | py::array::forcecast> AsArray(
    const py::array& a) {
  auto out =
      py::array_t<double, py::array::c_style | py::array::forcecast>(a);
  if (out.ndim() != N) {
    throw std::runtime_error("Expected array with " + std::to_string(N) +
                             " dimensions");
  }
  return out;
}

py::array_t<double, py::array::f_style | py::array::forcecast> AsFArray2(
    const py::array& a) {
  auto out =
      py::array_t<double, py::array::f_style | py::array::forcecast>(a);
  if (out.ndim() != 2) {
    throw std::runtime_error("Expected 2D array");
  }
  return out;
}

py::array_t<double, py::array::f_style | py::array::forcecast> AsFArray3(
    const py::array& a) {
  auto out =
      py::array_t<double, py::array::f_style | py::array::forcecast>(a);
  if (out.ndim() != 3) {
    throw std::runtime_error("Expected 3D array");
  }
  return out;
}

py::array_t<double, py::array::c_style | py::array::forcecast> AsMutable1D(
    py::array& a) {
  auto out = py::array_t<double, py::array::c_style | py::array::forcecast>(a);
  if (out.ndim() != 1) {
    throw std::runtime_error("Expected 1D array");
  }
  return out;
}

py::array_t<double, py::array::c_style | py::array::forcecast> AsMutable2D(
    py::array& a) {
  auto out = py::array_t<double, py::array::c_style | py::array::forcecast>(a);
  if (out.ndim() != 2) {
    throw std::runtime_error("Expected 2D array");
  }
  return out;
}

Eigen::VectorXd ToEigenVector(const py::array& a) {
  auto vec = AsArray<1>(a);
  auto vi = vec.request();
  const auto* vptr = static_cast<const double*>(vi.ptr);
  const ssize_t stride = vi.strides[0] / static_cast<ssize_t>(sizeof(double));
  Eigen::VectorXd out(static_cast<int>(vi.shape[0]));
  for (int i = 0; i < out.rows(); ++i) {
    out(i) = vptr[i * stride];
  }
  return out;
}

py::array_t<double> ToPyArray(const Eigen::VectorXd& v) {
  py::array_t<double> out({v.rows()}, {static_cast<ssize_t>(sizeof(double))});
  auto oi = out.request();
  auto* optr = static_cast<double*>(oi.ptr);
  for (int i = 0; i < v.rows(); ++i) {
    optr[i] = v(i);
  }
  return out;
}

using RowSparseMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor>;

struct SparseLSTiming {
  double support_ms = 0.0;
  double blocks_ms = 0.0;
  double finalize_ms = 0.0;
  double factor_ms = 0.0;
  double solve_ms = 0.0;
  double total_ms = 0.0;
  int num_cliques = 0;
  bool single_dense_clique = false;
};

class ScopedEigenNoMalloc final {
 public:
  ScopedEigenNoMalloc() {
#if defined(EIGEN_RUNTIME_NO_MALLOC)
    previous_state_ = Eigen::internal::is_malloc_allowed();
    Eigen::internal::set_is_malloc_allowed(false);
#endif
  }

  ~ScopedEigenNoMalloc() {
#if defined(EIGEN_RUNTIME_NO_MALLOC)
    Eigen::internal::set_is_malloc_allowed(previous_state_);
#endif
  }

 private:
#if defined(EIGEN_RUNTIME_NO_MALLOC)
  bool previous_state_ = true;
#endif
};

class StaticMatrixAssembler final : public conex::SupernodalAssemblerBase {
 public:
  StaticMatrixAssembler(const std::vector<int>& variables,
                        const Eigen::MatrixXd& local_matrix)
      : conex::SupernodalAssemblerBase(variables), local_matrix_(local_matrix) {
    if (local_matrix_.rows() != local_matrix_.cols()) {
      throw std::runtime_error("Local matrix must be square.");
    }
    if (local_matrix_.rows() != static_cast<int>(variables.size())) {
      throw std::runtime_error("Local matrix size must match variables.");
    }
    conex::Workspace workspace(&submatrix_data_);
    memory_.resize(SizeOf(workspace));
    Initialize(&workspace, memory_.data());
  }

  void SetDenseData() override {
    submatrix_data_.G.setZero();
    submatrix_data_.G.triangularView<Eigen::Lower>() =
        local_matrix_.triangularView<Eigen::Lower>();
  }

  void SetLocalMatrix(const Eigen::MatrixXd& local_matrix) {
    if (local_matrix.rows() != local_matrix.cols()) {
      throw std::runtime_error("Local matrix must be square.");
    }
    if (local_matrix.rows() != static_cast<int>(variables().size())) {
      throw std::runtime_error("Local matrix size must match variables.");
    }
    local_matrix_ = local_matrix;
  }

 private:
  Eigen::MatrixXd local_matrix_;
  Eigen::VectorXd memory_;
};

std::vector<int> VectorFromPyInt64(const py::array& arr, const char* name) {
  auto vec = py::array_t<long long, py::array::c_style | py::array::forcecast>(arr);
  if (vec.ndim() != 1) {
    throw std::runtime_error(std::string(name) + " must be a 1D array.");
  }
  std::vector<int> out(static_cast<size_t>(vec.shape(0)));
  auto u = vec.unchecked<1>();
  for (ssize_t i = 0; i < vec.shape(0); ++i) {
    out.at(static_cast<size_t>(i)) = static_cast<int>(u(i));
  }
  return out;
}

std::vector<std::vector<int>> NestedVectorFromPy(const py::list& nested,
                                                 const char* name) {
  std::vector<std::vector<int>> out;
  out.reserve(nested.size());
  for (size_t i = 0; i < nested.size(); ++i) {
    auto arr = py::array(nested[i]);
    out.push_back(VectorFromPyInt64(arr, name));
  }
  return out;
}

py::array_t<double> MatrixToPyArray(const Eigen::MatrixXd& mat) {
  py::array_t<double> out({mat.rows(), mat.cols()});
  auto o = out.mutable_unchecked<2>();
  for (int r = 0; r < mat.rows(); ++r) {
    for (int c = 0; c < mat.cols(); ++c) {
      o(r, c) = mat(r, c);
    }
  }
  return out;
}

Eigen::MatrixXd MatrixFromPy(const py::array& a, const char* name) {
  auto mat = py::array_t<double, py::array::c_style | py::array::forcecast>(a);
  if (mat.ndim() != 2) {
    throw std::runtime_error(std::string(name) + " must be a 2D array.");
  }
  auto info = mat.request();
  const auto* ptr = static_cast<const double*>(info.ptr);
  const ssize_t rs = info.strides[0] / static_cast<ssize_t>(sizeof(double));
  const ssize_t cs = info.strides[1] / static_cast<ssize_t>(sizeof(double));
  Eigen::MatrixXd out(static_cast<int>(info.shape[0]), static_cast<int>(info.shape[1]));
  for (int r = 0; r < out.rows(); ++r) {
    for (int c = 0; c < out.cols(); ++c) {
      out(r, c) = ptr[r * rs + c * cs];
    }
  }
  return out;
}

class PyKKTTreeSolver {
 public:
  PyKKTTreeSolver() { solver_.EnableAutoUpdateAtAssemble(true); }
  void SetNumThreads(int num_threads) { solver_.SetNumThreads(num_threads); }
  void SetParallelizeRootsOnly(bool enable) {
    solver_.SetParallelizeRootsOnly(enable);
  }
  void SetFactorizationMode(bool left_looking) {
    solver_.SetFactorizationMode(left_looking);
  }
  void EnableAutoUpdateAtAssemble(bool enable) {
    solver_.EnableAutoUpdateAtAssemble(enable);
  }

  int AddDenseSubsystem(const py::array& variables, const py::array& local_matrix) {
    std::vector<int> vars = VectorFromPyInt64(variables, "variables");
    Eigen::MatrixXd local = MatrixFromPy(local_matrix, "local_matrix");
    assemblers_.push_back(std::make_unique<StaticMatrixAssembler>(vars, local));
    auto adapter =
        std::make_unique<conex::KKTAssemblerToSubsystemAdapter>(assemblers_.back().get());
    auto* subsystem =
        adapter->create_subsystem(conex::SubsystemType::kPositiveDefinite);
    solver_.AddSubsystem(subsystem);
    solver_.push_back(std::move(adapter));
    return static_cast<int>(assemblers_.size()) - 1;
  }

  void UpdateLocalMatrix(int subsystem_index, const py::array& local_matrix) {
    if (subsystem_index < 0 ||
        subsystem_index >= static_cast<int>(assemblers_.size())) {
      throw std::runtime_error("subsystem_index out of range.");
    }
    assemblers_.at(static_cast<size_t>(subsystem_index))
        ->SetLocalMatrix(MatrixFromPy(local_matrix, "local_matrix"));
  }

  void Finalize(const py::list& supernodes, const py::list& separators,
                const py::array& node_to_parent) {
    conex::CliqueTree tree;
    tree.supernodes = NestedVectorFromPy(supernodes, "supernodes");
    tree.separators = NestedVectorFromPy(separators, "separators");
    tree.node_to_parent = VectorFromPyInt64(node_to_parent, "node_to_parent");
    const size_t n = assemblers_.size();
    if (tree.supernodes.size() != n || tree.separators.size() != n ||
        tree.node_to_parent.size() != n) {
      throw std::runtime_error(
          "Finalize expects supernodes/separators/node_to_parent sized to "
          "number of added subsystems.");
    }
    solver_.Finalize(tree);
  }

  void UpdateAssemblerData() { solver_.UpdateAssemblerData(); }
  void Assemble() {
    solver_.UpdateAssemblerData();
    solver_.Assemble();
  }
  bool Factor() {
    solver_.UpdateAssemblerData();
    return solver_.Factor();
  }
  bool AssembleAndFactor() {
    solver_.UpdateAssemblerData();
    return solver_.AssembleAndFactor();
  }

  py::array_t<double> Solve(const py::array& rhs, bool in_original_order) const {
    Eigen::MatrixXd b = MatrixFromPy(rhs, "rhs");
    if (b.rows() != solver_.number_of_variables()) {
      throw std::runtime_error("rhs row count must match solver variable count.");
    }
    Eigen::MatrixXd x;
    {
      py::gil_scoped_release release;
      x = solver_.Solve(b, in_original_order);
    }
    return MatrixToPyArray(x);
  }

  py::array_t<double> KKTMatrix(bool in_elimination_order) const {
    Eigen::MatrixXd M = solver_.KKTMatrix(in_elimination_order);
    return MatrixToPyArray(M);
  }

  py::list VariableToEliminationPosition() const {
    const auto& order = solver_.variable_to_elimination_position();
    py::list out;
    for (size_t i = 0; i < order.size(); ++i) {
      out.append(order.at(i));
    }
    return out;
  }

 private:
  conex::SymmetricLinearSystemTreeSolver solver_;
  std::vector<std::unique_ptr<StaticMatrixAssembler>> assemblers_;
};

std::vector<int> RowSupport(const RowSparseMatrix& A, int row) {
  std::vector<int> support;
  for (RowSparseMatrix::InnerIterator it(A, row); it; ++it) {
    support.push_back(it.col());
  }
  std::sort(support.begin(), support.end());
  support.erase(std::unique(support.begin(), support.end()), support.end());
  return support;
}

RowSparseMatrix BuildSparseFromCSR(const py::array& indptr, const py::array& indices,
                                   const py::array& data, int m, int n) {
  auto indptr_arr =
      py::array_t<long long, py::array::c_style | py::array::forcecast>(indptr);
  auto indices_arr =
      py::array_t<long long, py::array::c_style | py::array::forcecast>(indices);
  auto data_arr =
      py::array_t<double, py::array::c_style | py::array::forcecast>(data);
  if (indptr_arr.ndim() != 1 || indices_arr.ndim() != 1 || data_arr.ndim() != 1) {
    throw std::runtime_error("indptr, indices, data must be 1D arrays.");
  }
  if (indptr_arr.shape(0) != m + 1) {
    throw std::runtime_error("indptr length must equal m + 1.");
  }
  if (indices_arr.shape(0) != data_arr.shape(0)) {
    throw std::runtime_error("indices and data lengths must match.");
  }
  auto indptr_u = indptr_arr.unchecked<1>();
  auto indices_u = indices_arr.unchecked<1>();
  auto data_u = data_arr.unchecked<1>();
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(data_arr.shape(0)));
  for (int row = 0; row < m; ++row) {
    const long long start = indptr_u(row);
    const long long end = indptr_u(row + 1);
    for (long long k = start; k < end; ++k) {
      const int col = static_cast<int>(indices_u(k));
      if (col < 0 || col >= n) {
        throw std::runtime_error("indices out of bounds for shape.");
      }
      triplets.emplace_back(row, col, data_u(k));
    }
  }
  RowSparseMatrix A(m, n);
  A.setFromTriplets(triplets.begin(), triplets.end());
  A.makeCompressed();
  return A;
}

RowSparseMatrix BuildSparseFromCOO(const py::array& rows, const py::array& cols,
                                   const py::array& data, int m, int n) {
  auto rows_arr =
      py::array_t<long long, py::array::c_style | py::array::forcecast>(rows);
  auto cols_arr =
      py::array_t<long long, py::array::c_style | py::array::forcecast>(cols);
  auto data_arr =
      py::array_t<double, py::array::c_style | py::array::forcecast>(data);
  if (rows_arr.ndim() != 1 || cols_arr.ndim() != 1 || data_arr.ndim() != 1) {
    throw std::runtime_error("rows, cols, data must be 1D arrays.");
  }
  if (rows_arr.shape(0) != cols_arr.shape(0) || rows_arr.shape(0) != data_arr.shape(0)) {
    throw std::runtime_error("rows/cols/data must have same length.");
  }
  auto rows_u = rows_arr.unchecked<1>();
  auto cols_u = cols_arr.unchecked<1>();
  auto data_u = data_arr.unchecked<1>();
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(data_arr.shape(0)));
  for (ssize_t k = 0; k < data_arr.shape(0); ++k) {
    const int r = static_cast<int>(rows_u(k));
    const int c = static_cast<int>(cols_u(k));
    if (r < 0 || r >= m || c < 0 || c >= n) {
      throw std::runtime_error("COO row/col out of bounds.");
    }
    triplets.emplace_back(r, c, data_u(k));
  }
  RowSparseMatrix A(m, n);
  A.setFromTriplets(triplets.begin(), triplets.end());
  A.makeCompressed();
  return A;
}

RowSparseMatrix BuildSparseFromDense(const py::array& A_dense) {
  auto A = py::array_t<double, py::array::c_style | py::array::forcecast>(A_dense);
  if (A.ndim() != 2) {
    throw std::runtime_error("A must be 2D.");
  }
  auto ai = A.request();
  const int m = static_cast<int>(ai.shape[0]);
  const int n = static_cast<int>(ai.shape[1]);
  const auto* ptr = static_cast<double*>(ai.ptr);
  const ssize_t rs = ai.strides[0] / static_cast<ssize_t>(sizeof(double));
  const ssize_t cs = ai.strides[1] / static_cast<ssize_t>(sizeof(double));
  std::vector<Eigen::Triplet<double>> triplets;
  for (int r = 0; r < m; ++r) {
    for (int c = 0; c < n; ++c) {
      double v = ptr[r * rs + c * cs];
      if (v != 0.0) {
        triplets.emplace_back(r, c, v);
      }
    }
  }
  RowSparseMatrix S(m, n);
  S.setFromTriplets(triplets.begin(), triplets.end());
  S.makeCompressed();
  return S;
}

Eigen::VectorXd SparseLeastSquaresViaTree(const RowSparseMatrix& A,
                                          const Eigen::VectorXd& b,
                                          int num_threads,
                                          SparseLSTiming* timing = nullptr) {
  const auto total_start = std::chrono::steady_clock::now();
  if (A.rows() != b.rows()) {
    throw std::runtime_error("A and b dimension mismatch.");
  }
  const auto support_start = std::chrono::steady_clock::now();
  std::map<std::vector<int>, std::vector<int>> grouped_rows;
  for (int row = 0; row < A.rows(); ++row) {
    auto support = RowSupport(A, row);
    if (!support.empty()) {
      grouped_rows[support].push_back(row);
    }
  }
  const auto blocks_start = std::chrono::steady_clock::now();
  std::vector<std::vector<int>> cliques;
  std::vector<Eigen::MatrixXd> local_blocks;
  cliques.reserve(grouped_rows.size() + static_cast<size_t>(A.cols()));
  local_blocks.reserve(grouped_rows.size() + static_cast<size_t>(A.cols()));

  std::vector<char> variable_covered(static_cast<size_t>(A.cols()), 0);
  const bool single_dense_clique =
      grouped_rows.size() == 1 &&
      grouped_rows.begin()->first.size() == static_cast<size_t>(A.cols()) &&
      grouped_rows.begin()->second.size() == static_cast<size_t>(A.rows());
  if (single_dense_clique) {
    const auto& vars = grouped_rows.begin()->first;
    cliques.push_back(vars);
    Eigen::SparseMatrix<double> A_col_major = A;
    local_blocks.push_back(Eigen::MatrixXd(A_col_major.transpose() * A_col_major));
    std::fill(variable_covered.begin(), variable_covered.end(), 1);
  } else {
    for (const auto& entry : grouped_rows) {
      const auto& vars = entry.first;
      const auto& rows = entry.second;
      for (int col : vars) {
        variable_covered.at(static_cast<size_t>(col)) = 1;
      }
      Eigen::MatrixXd ag(rows.size(), vars.size());
      ag.setZero();
      for (size_t r = 0; r < rows.size(); ++r) {
        for (RowSparseMatrix::InnerIterator it(A, rows.at(r)); it; ++it) {
          auto lb = std::lower_bound(vars.begin(), vars.end(), it.col());
          if (lb != vars.end() && *lb == it.col()) {
            const int c = static_cast<int>(lb - vars.begin());
            ag(static_cast<int>(r), c) = it.value();
          }
        }
      }
      Eigen::MatrixXd block = ag.transpose() * ag;
      cliques.push_back(vars);
      local_blocks.push_back(std::move(block));
    }
  }

  // Include uncovered variables with tiny ridge to keep the system SPD.
  for (int col = 0; col < A.cols(); ++col) {
    if (!variable_covered.at(static_cast<size_t>(col))) {
      cliques.push_back({col});
      Eigen::MatrixXd block(1, 1);
      block(0, 0) = 1e-12;
      local_blocks.push_back(std::move(block));
    }
  }
  if (cliques.empty()) {
    throw std::runtime_error("No cliques constructed from A.");
  }
  const auto finalize_start = std::chrono::steady_clock::now();
  conex::CliqueTree clique_tree = conex::MakeCliqueTree(cliques);

  conex::SymmetricLinearSystemTreeSolver tree_solver;
  tree_solver.SetNumThreads(single_dense_clique ? 1 : std::max(1, num_threads));
  tree_solver.EnableAutoUpdateAtAssemble(true);
  tree_solver.SetFactorizationMode(true);

  std::vector<std::unique_ptr<StaticMatrixAssembler>> assemblers;
  std::vector<std::unique_ptr<conex::KKTAssemblerToSubsystemAdapter>> adapters;
  assemblers.reserve(cliques.size());
  adapters.reserve(cliques.size());

  for (size_t i = 0; i < cliques.size(); ++i) {
    assemblers.emplace_back(std::make_unique<StaticMatrixAssembler>(
        cliques.at(i), 2.0 * local_blocks.at(i)));
    auto adapter = std::make_unique<conex::KKTAssemblerToSubsystemAdapter>(
        assemblers.back().get());
    auto* subsystem =
        adapter->create_subsystem(conex::SubsystemType::kPositiveDefinite);
    tree_solver.AddSubsystem(subsystem);
    tree_solver.push_back(std::move(adapter));
  }
  tree_solver.Finalize(clique_tree);
  const auto factor_start = std::chrono::steady_clock::now();
  tree_solver.Assemble();
  {
    ScopedEigenNoMalloc no_malloc_during_factor;
    if (!tree_solver.Factor()) {
      throw std::runtime_error("Conex tree least-squares factorization failed.");
    }
  }
  const auto solve_start = std::chrono::steady_clock::now();
  Eigen::VectorXd rhs = 2.0 * (A.transpose() * b);
  Eigen::MatrixXd rhs_mat(rhs.rows(), 1);
  rhs_mat.col(0) = rhs;
  const auto& variable_to_elimination_position =
      tree_solver.variable_to_elimination_position();
  if (static_cast<int>(variable_to_elimination_position.size()) != rhs.rows()) {
    throw std::runtime_error("Invalid elimination permutation size.");
  }
  Eigen::PermutationMatrix<-1> P(rhs.rows());
  P.indices() = Eigen::Map<const Eigen::VectorXi>(
      variable_to_elimination_position.data(), rhs.rows());
  Eigen::MatrixXd x = P * rhs_mat;
  tree_solver.ReserveSolveWorkspace(x.cols());
  {
    ScopedEigenNoMalloc no_malloc_during_solve;
    tree_solver.SolveInPlace(x, false);
  }
  x = P.transpose() * x;
  const auto end = std::chrono::steady_clock::now();
  if (timing != nullptr) {
    timing->support_ms = std::chrono::duration<double, std::milli>(
                             blocks_start - support_start)
                             .count();
    timing->blocks_ms = std::chrono::duration<double, std::milli>(
                            finalize_start - blocks_start)
                            .count();
    timing->finalize_ms = std::chrono::duration<double, std::milli>(
                              factor_start - finalize_start)
                              .count();
    timing->factor_ms = std::chrono::duration<double, std::milli>(
                            solve_start - factor_start)
                            .count();
    timing->solve_ms =
        std::chrono::duration<double, std::milli>(end - solve_start).count();
    timing->total_ms =
        std::chrono::duration<double, std::milli>(end - total_start).count();
    timing->num_cliques = static_cast<int>(cliques.size());
    timing->single_dense_clique = single_dense_clique;
  }
  return x.col(0);
}

}  // namespace

PYBIND11_MODULE(_conex, m) {
  m.attr("CLIQUE_TREE_METHOD_WEIGHTED_DFS") =
      py::int_(static_cast<int>(conex::CLIQUE_TREE_METHOD_WEIGHTED_DFS));
  m.attr("CLIQUE_TREE_METHOD_AMD") =
      py::int_(static_cast<int>(conex::CLIQUE_TREE_METHOD_AMD));

  m.def("build_clique_tree",
        [](const py::list& cliques, int method) {
          conex::CliqueTree tree =
              conex::MakeCliqueTree(NestedVectorFromPy(cliques, "cliques"), {},
                                    method);
          py::dict out;
          out["supernodes"] = py::cast(tree.supernodes);
          out["separators"] = py::cast(tree.separators);
          out["node_to_parent"] = py::cast(tree.node_to_parent);
          out["order_to_clique"] =
              py::cast(tree.post_order_position_to_clique);
          return out;
        },
        py::arg("cliques"),
        py::arg("method") = static_cast<int>(conex::CLIQUE_TREE_METHOD_AMD));

  m.def("build_primal_dual_clique_tree",
        [](const py::list& cliques, const py::list& dual_variables,
           int method) {
          conex::CliqueTree tree = conex::MakePrimalDualCliqueTree(
              NestedVectorFromPy(cliques, "cliques"),
              NestedVectorFromPy(dual_variables, "dual_variables"), method);
          py::dict out;
          out["supernodes"] = py::cast(tree.supernodes);
          out["separators"] = py::cast(tree.separators);
          out["node_to_parent"] = py::cast(tree.node_to_parent);
          out["order_to_clique"] =
              py::cast(tree.post_order_position_to_clique);
          return out;
        },
        py::arg("cliques"), py::arg("dual_variables"),
        py::arg("method") = static_cast<int>(conex::CLIQUE_TREE_METHOD_AMD));

  py::class_<PyKKTTreeSolver>(m, "KKTTreeSolver")
      .def(py::init<>())
      .def("set_num_threads", &PyKKTTreeSolver::SetNumThreads,
           py::arg("num_threads"))
      .def("set_parallelize_roots_only",
           &PyKKTTreeSolver::SetParallelizeRootsOnly, py::arg("enable"))
      .def("set_factorization_mode", &PyKKTTreeSolver::SetFactorizationMode,
           py::arg("left_looking"))
      .def("enable_auto_update_at_assemble",
           &PyKKTTreeSolver::EnableAutoUpdateAtAssemble, py::arg("enable"))
      .def("add_dense_subsystem", &PyKKTTreeSolver::AddDenseSubsystem,
           py::arg("variables"), py::arg("local_matrix"))
      .def("update_local_matrix", &PyKKTTreeSolver::UpdateLocalMatrix,
           py::arg("subsystem_index"), py::arg("local_matrix"))
      .def("finalize", &PyKKTTreeSolver::Finalize, py::arg("supernodes"),
           py::arg("separators"), py::arg("node_to_parent"))
      .def("update_assembler_data", &PyKKTTreeSolver::UpdateAssemblerData)
      .def("assemble", &PyKKTTreeSolver::Assemble)
      .def("factor", &PyKKTTreeSolver::Factor)
      .def("assemble_and_factor", &PyKKTTreeSolver::AssembleAndFactor)
      .def("solve", &PyKKTTreeSolver::Solve, py::arg("rhs"),
           py::arg("in_original_order") = true)
      .def("kkt_matrix", &PyKKTTreeSolver::KKTMatrix,
           py::arg("in_elimination_order") = false)
      .def("variable_to_elimination_position",
           &PyKKTTreeSolver::VariableToEliminationPosition);

  py::class_<IntPtr>(m, "intp")
      .def(py::init<int>(), py::arg("value") = 0)
      .def("value", &IntPtr::value)
      .def("set", &IntPtr::set);

  py::class_<CONEX_SolverConfiguration>(m, "CONEX_SolverConfiguration")
      .def(py::init<>())
      .def_readwrite("prepare_dual_variables",
                     &CONEX_SolverConfiguration::prepare_dual_variables)
      .def_readwrite("initialization_mode",
                     &CONEX_SolverConfiguration::initialization_mode)
      .def_readwrite("inv_sqrt_mu_max",
                     &CONEX_SolverConfiguration::inv_sqrt_mu_max)
      .def_readwrite("minimum_mu", &CONEX_SolverConfiguration::minimum_mu)
      .def_readwrite("maximum_mu", &CONEX_SolverConfiguration::maximum_mu)
      .def_readwrite("divergence_upper_bound",
                     &CONEX_SolverConfiguration::divergence_upper_bound)
      .def_readwrite("enable_line_search",
                     &CONEX_SolverConfiguration::enable_line_search)
      .def_readwrite("dinf_upper_bound",
                     &CONEX_SolverConfiguration::dinf_upper_bound)
      .def_readwrite("final_centering_steps",
                     &CONEX_SolverConfiguration::final_centering_steps)
      .def_readwrite("final_centering_tolerance",
                     &CONEX_SolverConfiguration::final_centering_tolerance)
      .def_readwrite("initial_centering_steps_warmstart",
                     &CONEX_SolverConfiguration::initial_centering_steps_warmstart)
      .def_readwrite("initial_centering_steps_coldstart",
                     &CONEX_SolverConfiguration::initial_centering_steps_coldstart)
      .def_readwrite("warmstart_abort_threshold",
                     &CONEX_SolverConfiguration::warmstart_abort_threshold)
      .def_readwrite("max_iterations", &CONEX_SolverConfiguration::max_iterations)
      .def_readwrite("iterative_refinement_iterations",
                     &CONEX_SolverConfiguration::iterative_refinement_iterations)
      .def_readwrite("num_threads", &CONEX_SolverConfiguration::num_threads)
      .def_readwrite("infeasibility_threshold",
                     &CONEX_SolverConfiguration::infeasibility_threshold)
      .def_readwrite("kkt_error_tolerance",
                     &CONEX_SolverConfiguration::kkt_error_tolerance)
      .def_readwrite("enable_rescaling",
                     &CONEX_SolverConfiguration::enable_rescaling)
      .def_readwrite("enable_scale_correction",
                     &CONEX_SolverConfiguration::enable_scale_correction)
      .def_readwrite("kkt_solver", &CONEX_SolverConfiguration::kkt_solver)
      .def_readwrite("clique_tree_method",
                     &CONEX_SolverConfiguration::clique_tree_method)
      .def_readwrite("verbosity", &CONEX_SolverConfiguration::verbosity);

  py::class_<CONEX_IterationStats>(m, "CONEX_IterationStats")
      .def(py::init<>())
      .def_readwrite("mu", &CONEX_IterationStats::mu)
      .def_readwrite("iteration_number", &CONEX_IterationStats::iteration_number);

  py::class_<CONEX_SolutionStats>(m, "CONEX_SolutionStats")
      .def(py::init<>())
      .def_readwrite("iterations", &CONEX_SolutionStats::iterations)
      .def_readwrite("duality_gap", &CONEX_SolutionStats::duality_gap);

  m.def("CONEX_CreateConeProgram",
        []() { return PtrToPy(CONEX_CreateConeProgram()); });
  m.def("CONEX_DeleteConeProgram",
        [](std::uintptr_t p) { CONEX_DeleteConeProgram(PtrFromPy(p)); });
  m.def("CONEX_SetNumberOfVariables", [](std::uintptr_t p, int n) {
    return CONEX_SetNumberOfVariables(PtrFromPy(p), n);
  });
  m.def("CONEX_SetDefaultOptions",
        [](CONEX_SolverConfiguration& cfg) { CONEX_SetDefaultOptions(&cfg); });

  m.def("CONEX_AddDenseLinearConstraint",
        [](std::uintptr_t p, const py::array& A, const py::array& c) {
          auto a = AsFArray2(A);
          auto cv = AsArray<1>(c);
          auto ai = a.request();
          auto ci = cv.request();
          return CONEX_AddDenseLinearConstraint(
              PtrFromPy(p), static_cast<double*>(ai.ptr), static_cast<int>(ai.shape[0]),
              static_cast<int>(ai.shape[1]), static_cast<double*>(ci.ptr),
              static_cast<int>(ci.shape[0]));
        });

  m.def("CONEX_AddLinearInequalities",
        [](std::uintptr_t p, const py::array& A, const py::array& lb,
           const py::array& ub) {
          auto a = AsFArray2(A);
          auto l = AsArray<1>(lb);
          auto u = AsArray<1>(ub);
          auto ai = a.request();
          auto li = l.request();
          auto ui = u.request();
          return CONEX_AddLinearInequalities(
              PtrFromPy(p), static_cast<double*>(ai.ptr), static_cast<int>(ai.shape[0]),
              static_cast<int>(ai.shape[1]), static_cast<double*>(li.ptr),
              static_cast<int>(li.shape[0]), static_cast<double*>(ui.ptr),
              static_cast<int>(ui.shape[0]));
        });

  m.def("CONEX_AddQuadraticCost", [](std::uintptr_t p, const py::array& A) {
    auto a = AsFArray2(A);
    auto ai = a.request();
    return CONEX_AddQuadraticCost(PtrFromPy(p), static_cast<double*>(ai.ptr),
                                  static_cast<int>(ai.shape[0]),
                                  static_cast<int>(ai.shape[1]));
  });

  m.def("CONEX_AddDenseLMIConstraint",
        [](std::uintptr_t p, const py::array& Aarray, const py::array& cmat) {
          auto a = AsFArray3(Aarray);
          auto c = AsFArray2(cmat);
          auto ai = a.request();
          auto ci = c.request();
          return CONEX_AddDenseLMIConstraint(
              PtrFromPy(p), static_cast<double*>(ai.ptr), static_cast<int>(ai.shape[0]),
              static_cast<int>(ai.shape[1]), static_cast<int>(ai.shape[2]),
              static_cast<double*>(ci.ptr), static_cast<int>(ci.shape[0]),
              static_cast<int>(ci.shape[1]));
        });

  m.def("CONEX_AddSparseLMIConstraint",
        [](std::uintptr_t p, const py::array& Aarray, const py::array& cmat,
           const py::array& vars) {
          auto a = AsFArray3(Aarray);
          auto c = AsFArray2(cmat);
          auto v = py::array_t<long, py::array::forcecast>(vars);
          if (v.ndim() != 1) throw std::runtime_error("Expected 1D vars array");
          auto ai = a.request();
          auto ci = c.request();
          auto vi = v.request();
          return CONEX_AddSparseLMIConstraint(
              PtrFromPy(p), static_cast<double*>(ai.ptr), static_cast<int>(ai.shape[0]),
              static_cast<int>(ai.shape[1]), static_cast<int>(ai.shape[2]),
              static_cast<double*>(ci.ptr), static_cast<int>(ci.shape[0]),
              static_cast<int>(ci.shape[1]), static_cast<long*>(vi.ptr),
              static_cast<int>(vi.shape[0]));
        });

  m.def("CONEX_AddLinearCost", [](std::uintptr_t p, const py::array& b) {
    auto bv = AsArray<1>(b);
    auto bi = bv.request();
    return CONEX_AddLinearCost(PtrFromPy(p), static_cast<double*>(bi.ptr),
                               static_cast<int>(bi.shape[0]));
  });

  m.def("CONEX_Solve",
        [](std::uintptr_t p, const CONEX_SolverConfiguration& cfg,
           py::array y) {
          auto yy = AsMutable1D(y);
          auto yi = yy.request();
          py::gil_scoped_release release;
          return CONEX_Solve(PtrFromPy(p), &cfg, static_cast<double*>(yi.ptr),
                             static_cast<int>(yi.shape[0]));
        });

  m.def(
      "sparse_ls_profile_csr",
      [](const py::array& indptr, const py::array& indices, const py::array& data,
         int m_rows, int n_cols, const py::array& b, int num_threads) {
        Eigen::VectorXd b_eig = ToEigenVector(b);
        if (b_eig.rows() != m_rows) {
          throw std::runtime_error("b length must equal number of rows in A.");
        }
        RowSparseMatrix A =
            BuildSparseFromCSR(indptr, indices, data, m_rows, n_cols);
        SparseLSTiming timing;
        Eigen::VectorXd x;
        {
          py::gil_scoped_release release;
          x = SparseLeastSquaresViaTree(A, b_eig, num_threads, &timing);
        }
        py::dict out;
        out["x"] = ToPyArray(x);
        out["support_ms"] = timing.support_ms;
        out["blocks_ms"] = timing.blocks_ms;
        out["finalize_ms"] = timing.finalize_ms;
        out["factor_ms"] = timing.factor_ms;
        out["solve_ms"] = timing.solve_ms;
        out["total_ms"] = timing.total_ms;
        out["num_cliques"] = timing.num_cliques;
        out["single_dense_clique"] = timing.single_dense_clique;
        return out;
      },
      py::arg("indptr"), py::arg("indices"), py::arg("data"), py::arg("m_rows"),
      py::arg("n_cols"), py::arg("b"), py::arg("num_threads") = 1);

  m.def(
      "sparse_ls_csr",
      [](const py::array& indptr, const py::array& indices, const py::array& data,
         int m_rows, int n_cols, const py::array& b, int num_threads) {
        Eigen::VectorXd b_eig = ToEigenVector(b);
        if (b_eig.rows() != m_rows) {
          throw std::runtime_error("b length must equal number of rows in A.");
        }
        RowSparseMatrix A =
            BuildSparseFromCSR(indptr, indices, data, m_rows, n_cols);
        Eigen::VectorXd x;
        {
          py::gil_scoped_release release;
          x = SparseLeastSquaresViaTree(A, b_eig, num_threads);
        }
        return ToPyArray(x);
      },
      py::arg("indptr"), py::arg("indices"), py::arg("data"), py::arg("m_rows"),
      py::arg("n_cols"), py::arg("b"), py::arg("num_threads") = 1);

  m.def(
      "sparse_ls_coo",
      [](const py::array& rows, const py::array& cols, const py::array& data,
         int m_rows, int n_cols, const py::array& b, int num_threads) {
        Eigen::VectorXd b_eig = ToEigenVector(b);
        if (b_eig.rows() != m_rows) {
          throw std::runtime_error("b length must equal number of rows in A.");
        }
        RowSparseMatrix A = BuildSparseFromCOO(rows, cols, data, m_rows, n_cols);
        Eigen::VectorXd x;
        {
          py::gil_scoped_release release;
          x = SparseLeastSquaresViaTree(A, b_eig, num_threads);
        }
        return ToPyArray(x);
      },
      py::arg("rows"), py::arg("cols"), py::arg("data"), py::arg("m_rows"),
      py::arg("n_cols"), py::arg("b"), py::arg("num_threads") = 1);

  m.def(
      "sparse_ls_dense",
      [](const py::array& A_dense, const py::array& b, int num_threads) {
        RowSparseMatrix A = BuildSparseFromDense(A_dense);
        Eigen::VectorXd b_eig = ToEigenVector(b);
        if (b_eig.rows() != A.rows()) {
          throw std::runtime_error("b length must equal number of rows in A.");
        }
        Eigen::VectorXd x;
        {
          py::gil_scoped_release release;
          x = SparseLeastSquaresViaTree(A, b_eig, num_threads);
        }
        return ToPyArray(x);
      },
      py::arg("A_dense"), py::arg("b"), py::arg("num_threads") = 1);

  m.def("CONEX_Maximize",
        [](std::uintptr_t p, const py::array& b, const CONEX_SolverConfiguration& cfg,
           py::array y) {
          auto bv = AsArray<1>(b);
          auto yy = AsMutable1D(y);
          auto bi = bv.request();
          auto yi = yy.request();
          py::gil_scoped_release release;
          return CONEX_Maximize(PtrFromPy(p), static_cast<double*>(bi.ptr),
                                static_cast<int>(bi.shape[0]), &cfg,
                                static_cast<double*>(yi.ptr),
                                static_cast<int>(yi.shape[0]));
        });

  m.def("CONEX_GetDualVariable", [](std::uintptr_t p, int i, py::array x) {
    auto xx = AsMutable2D(x);
    auto xi = xx.request();
    CONEX_GetDualVariable(PtrFromPy(p), i, static_cast<double*>(xi.ptr),
                          static_cast<int>(xi.shape[0]),
                          static_cast<int>(xi.shape[1]));
  });

  m.def("CONEX_GetIterationStats",
        [](std::uintptr_t p, CONEX_IterationStats& stats, int iter_num) {
          CONEX_GetIterationStats(PtrFromPy(p), &stats, iter_num);
        });

  m.def("CONEX_NewLinearMatrixInequality",
        [](std::uintptr_t p, int order, int hyper_complex_dim, IntPtr& out) {
          return CONEX_NewLinearMatrixInequality(PtrFromPy(p), order,
                                                 hyper_complex_dim, out.ptr());
        });
  m.def("CONEX_UpdateLinearOperator",
        [](std::uintptr_t p, int constraint, double value, int variable, int row,
           int col, int hyper_complex_dim) {
          return CONEX_UpdateLinearOperator(PtrFromPy(p), constraint, value, variable,
                                            row, col, hyper_complex_dim);
        });
  m.def("CONEX_UpdateAffineTerm",
        [](std::uintptr_t p, int constraint, double value, int row, int col,
           int hyper_complex_dim) {
          return CONEX_UpdateAffineTerm(PtrFromPy(p), constraint, value, row, col,
                                        hyper_complex_dim);
        });
  m.def("CONEX_NewLorentzConeConstraint",
        [](std::uintptr_t p, int order, IntPtr& out) {
          return CONEX_NewLorentzConeConstraint(PtrFromPy(p), order, out.ptr());
        });
  m.def("CONEX_NewLinearInequality",
        [](std::uintptr_t p, int rows, IntPtr& out) {
          return CONEX_NewLinearInequality(PtrFromPy(p), rows, out.ptr());
        });
  m.def("CONEX_NewQuadraticCost",
        [](std::uintptr_t p, IntPtr& out) {
          return CONEX_NewQuadraticCost(PtrFromPy(p), out.ptr());
        });
  m.def("CONEX_UpdateQuadraticCostMatrix",
        [](std::uintptr_t p, int id, double value, int row, int col) {
          return CONEX_UpdateQuadraticCostMatrix(PtrFromPy(p), id, value, row, col);
        });

  m.def("CONEX_QP_Solver",
        [](const py::array& Q, const py::array& cost, const py::array& ineq,
           const py::array& ub, const py::array& lb,
           const CONEX_SolverConfiguration& cfg, py::array solution,
           CONEX_SolutionStats& stats) {
          auto q = AsArray<2>(Q);
          auto qf = AsFArray2(Q);
          auto c = AsArray<1>(cost);
          auto a = AsFArray2(ineq);
          auto up = AsArray<1>(ub);
          auto lo = AsArray<1>(lb);
          auto y = AsMutable1D(solution);
          auto qi = qf.request();
          auto ci = c.request();
          auto ai = a.request();
          auto ui = up.request();
          auto li = lo.request();
          auto yi = y.request();
          py::gil_scoped_release release;
          return CONEX_QP_Solver(
              static_cast<double*>(qi.ptr), static_cast<int>(qi.shape[0]),
              static_cast<int>(qi.shape[1]), static_cast<double*>(ci.ptr),
              static_cast<int>(ci.shape[0]), static_cast<double*>(ai.ptr),
              static_cast<int>(ai.shape[0]), static_cast<int>(ai.shape[1]),
              static_cast<double*>(ui.ptr), static_cast<int>(ui.shape[0]),
              static_cast<double*>(li.ptr), static_cast<int>(li.shape[0]), &cfg,
              static_cast<double*>(yi.ptr), static_cast<int>(yi.shape[0]), &stats);
        });

  m.def("CONEX_QP_GetCanonicalProblemData",
        [](const py::array& Q, const py::array& cost, const py::array& ineq,
           const py::array& ub, const py::array& lb, IntPtr& num_ineq,
           IntPtr& num_eq, py::array matrix_A, py::array vector_b,
           py::array matrix_B, py::array vector_d) {
          auto q = AsArray<2>(Q);
          auto qf = AsFArray2(Q);
          auto c = AsArray<1>(cost);
          auto a = AsFArray2(ineq);
          auto up = AsArray<1>(ub);
          auto lo = AsArray<1>(lb);
          auto A = AsMutable2D(matrix_A);
          auto b = AsMutable1D(vector_b);
          auto B = AsMutable2D(matrix_B);
          auto d = AsMutable1D(vector_d);
          auto qi = qf.request();
          auto ci = c.request();
          auto ai = a.request();
          auto ui = up.request();
          auto li = lo.request();
          auto Ai = A.request();
          auto bi = b.request();
          auto Bi = B.request();
          auto di = d.request();
          return CONEX_QP_GetCanonicalProblemData(
              static_cast<double*>(qi.ptr), static_cast<int>(qi.shape[0]),
              static_cast<int>(qi.shape[1]), static_cast<double*>(ci.ptr),
              static_cast<int>(ci.shape[0]), static_cast<double*>(ai.ptr),
              static_cast<int>(ai.shape[0]), static_cast<int>(ai.shape[1]),
              static_cast<double*>(ui.ptr), static_cast<int>(ui.shape[0]),
              static_cast<double*>(li.ptr), static_cast<int>(li.shape[0]),
              num_ineq.ptr(), num_eq.ptr(), static_cast<double*>(Ai.ptr),
              static_cast<int>(Ai.shape[0]), static_cast<int>(Ai.shape[1]),
              static_cast<double*>(bi.ptr), static_cast<int>(bi.shape[0]),
              static_cast<double*>(Bi.ptr), static_cast<int>(Bi.shape[0]),
              static_cast<int>(Bi.shape[1]), static_cast<double*>(di.ptr),
              static_cast<int>(di.shape[0]));
        });
}
