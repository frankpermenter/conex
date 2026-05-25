#include "conex/common/tree_rhs.h"
#include "conex/common/arena.h"
#include <cstring>

namespace conex {


void SolverRHS::SetZero() {
  supernodes->SetZero();
  if (has_separators()) separators->SetZero();
  blocks_fully_gathered = true;
}

void SolverRHS::SetColumn(int col, const SolverRHS& src) {
  int nb = supernodes->num_blocks();
  for (int k = 0; k < nb; ++k)
    supernodes->block(k).col(col) = src.supernodes->block(k).col(0);
  if (has_separators() && src.has_separators()) {
    int nc = cols();
    int src_nc = src.cols();
    for (int k = 0; k < nb; ++k)
      separators->block(k, nc).col(col) =
          src.separators->block(k, src_nc).col(0);
  }
  if (!src.blocks_fully_gathered) blocks_fully_gathered = false;
}

namespace EuclideanJordanAlgebra {




Variable::Variable(const Variable& o)
    : offsets(o.offsets), sizes(o.sizes), ops(o.ops),
      data_(nullptr), rows_(o.rows_), cols_(o.cols_) {
  if (o.data_ && o.rows_ > 0 && o.cols_ > 0) {
    // Copy constructor has no buffer — must heap-allocate.
    heap_storage_ = std::shared_ptr<double[]>(new double[rows_ * cols_]);
    data_ = heap_storage_.get();
    std::memcpy(data_, o.data_, rows_ * cols_ * sizeof(double));
  }
}

Variable& Variable::operator=(const Variable& o) {
  if (this == &o) return *this;
  offsets = o.offsets;
  sizes = o.sizes;
  ops = o.ops;
  if (o.data_ && o.rows_ > 0 && o.cols_ > 0) {
    if (data_ && rows_ * cols_ >= o.rows_ * o.cols_) {
      // Reuse existing buffer (arena or heap) — no allocation.
      rows_ = o.rows_;
      cols_ = o.cols_;
    } else {
      // Need a larger buffer — heap-allocate.
      rows_ = o.rows_;
      cols_ = o.cols_;
      heap_storage_ = std::shared_ptr<double[]>(new double[rows_ * cols_]);
      data_ = heap_storage_.get();
    }
    std::memcpy(data_, o.data_, rows_ * cols_ * sizeof(double));
  } else {
    rows_ = o.rows_;
    cols_ = o.cols_;
    data_ = nullptr;
    heap_storage_ = nullptr;
  }
  return *this;
}

void Variable::resize(int rows, int ncols) {
  // Always allocate a fresh unique buffer (deep copy semantics).
  heap_storage_ = std::shared_ptr<double[]>(new double[rows * ncols]);
  data_ = heap_storage_.get();
  rows_ = rows;
  cols_ = ncols;
}

void Variable::setZero(int rows, int ncols) {
  resize(rows, ncols);
  std::memset(data_, 0, static_cast<size_t>(rows_) * cols_ * sizeof(double));
}

void Variable::BindArenaData(double* buf, const std::vector<int>& padded_offsets,
                              const std::vector<int>& seg_sizes, int ncols) {
  offsets = padded_offsets;
  sizes = seg_sizes;
  data_ = buf;
  constexpr int kAlignDoubles = Arena::kAlign / sizeof(double);
  int last_padded = seg_sizes.empty() ? 0 :
      (seg_sizes.back() + kAlignDoubles - 1) & ~(kAlignDoubles - 1);
  rows_ = padded_offsets.empty() ? 0 : padded_offsets.back() + last_padded;
  cols_ = ncols;
  heap_storage_ = nullptr;
}

Eigen::Map<Eigen::MatrixXd, 0, Eigen::OuterStride<>>
Variable::segment(int i) {
  return {data_ + offsets[i], sizes[i], cols_, Eigen::OuterStride<>(rows_)};
}

Eigen::Map<const Eigen::MatrixXd, 0, Eigen::OuterStride<>>
Variable::segment(int i) const {
  return {data_ + offsets[i], sizes[i], cols_, Eigen::OuterStride<>(rows_)};
}

Eigen::Map<Eigen::VectorXd> Variable::col(int c) {
  return {data_ + c * rows_, rows_};
}

Eigen::Map<const Eigen::VectorXd> Variable::col(int c) const {
  return {data_ + c * rows_, rows_};
}

void Variable::SetZero() {
  if (data_) std::memset(data_, 0, static_cast<size_t>(rows_) * cols_ * sizeof(double));
}

void Variable::SetScalarWeights(double val) {
  for (int i = 0; i < rows_ * cols_; ++i) data_[i] = val;
}

Variable& Variable::operator*=(double alpha) {
  for (int i = 0; i < rows_ * cols_; ++i) data_[i] *= alpha;
  return *this;
}

Variable& Variable::operator+=(const Variable& o) {
  for (int s = 0; s < num_constraints(); ++s) {
    int sz = sizes[s];
    for (int c = 0; c < cols_; ++c) {
      double* p = data_ + offsets[s] + c * rows_;
      const double* q = o.data_ + o.offsets[s] + c * o.rows_;
      for (int j = 0; j < sz; ++j) p[j] += q[j];
    }
  }
  return *this;
}

Variable& Variable::operator-=(const Variable& o) {
  for (int s = 0; s < num_constraints(); ++s) {
    int sz = sizes[s];
    for (int c = 0; c < cols_; ++c) {
      double* p = data_ + offsets[s] + c * rows_;
      const double* q = o.data_ + o.offsets[s] + c * o.rows_;
      for (int j = 0; j < sz; ++j) p[j] -= q[j];
    }
  }
  return *this;
}

// Deep copy: allocate new buffer and copy data.
static Variable deepCopy(const Variable& a) {
  Variable out;
  out.offsets = a.offsets;
  out.sizes = a.sizes;
  out.ops = a.ops;
  out.setZero(a.total_rows(), a.cols());
  std::memcpy(out.data(), a.data(), a.total_rows() * a.cols() * sizeof(double));
  return out;
}

Variable operator+(const Variable& a, const Variable& b) {
  Variable out = deepCopy(a); out += b; return out;
}

Variable operator-(const Variable& a, const Variable& b) {
  Variable out = deepCopy(a); out -= b; return out;
}

Variable operator*(double alpha, const Variable& a) {
  Variable out = deepCopy(a); out *= alpha; return out;
}

Variable operator*(const Variable& a, double alpha) {
  return alpha * a;
}

Variable operator*(const Variable& a, const Variable& b) {
  Variable out;
  out.offsets = a.offsets;
  out.sizes = a.sizes;
  out.ops = a.ops;
  out.setZero(a.total_rows(), a.cols());
  for (int i = 0; i < a.num_constraints(); ++i)
    static_cast<const SymmetricConeOperations*>(a.ops[i])->product(
        out.segment_ptr(i), a.segment_ptr(i),
        b.segment_ptr(i), a.sizes[i]);
  return out;
}

std::ostream& operator<<(std::ostream& os, const Variable& v) {
  for (int i = 0; i < v.num_constraints(); ++i) {
    int sz = v.sizes[i];
    int n = static_cast<int>(std::round(std::sqrt(static_cast<double>(sz))));
    bool is_square = (n * n == sz && n > 1);
    os << "segment " << i << " (" << sz << " entries)";
    if (is_square) {
      os << " [" << n << "x" << n << " matrix]:\n";
      Eigen::Map<const Eigen::MatrixXd> M(v.segment_ptr(i), n, n);
      os << M << "\n";
    } else {
      os << ":\n";
      Eigen::Map<const Eigen::VectorXd> vec(v.segment_ptr(i), sz);
      os << vec.transpose() << "\n";
    }
  }
  return os;
}

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
