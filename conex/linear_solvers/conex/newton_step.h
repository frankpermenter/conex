#pragma once
#include "debug_macros.h"
#include "memory_utils.h"
#include <Eigen/Dense>

namespace conex {

using DenseMatrix = Eigen::MatrixXd;
using Ref = Eigen::Map<DenseMatrix, Eigen::Aligned>;

using DenseMatrix = Eigen::MatrixXd;
struct WorkspaceSchurComplement {
  WorkspaceSchurComplement(int m) : m_(m) {}
  WorkspaceSchurComplement() {}

  static constexpr int size_of(int m, bool residual_only) {
    int size = 4 * get_size_aligned(m);
    if (!residual_only) {
      size += get_size_aligned(m * m);
    }
    return size;
  }

  friend int SizeOf(const WorkspaceSchurComplement& o) {
    return size_of(o.m_, o.residual_only_);
  }

  friend void Initialize(WorkspaceSchurComplement* o, double* data) {
    using Map = Eigen::Map<DenseMatrix, Eigen::Aligned>;
    int m = o->m_;
    new (&o->AW) Map(data, m, 1);
    new (&o->AQc) Map(data + 1 * get_size_aligned(m), m, 1);
    new (&o->AQe) Map(data + 2 * get_size_aligned(m), m, 1);
    new (&o->Ae) Map(data + 3 * get_size_aligned(m), m, 1);

    if (!o->residual_only_) {
      new (&o->G) Map(data + 4 * get_size_aligned(m), m, m);
    }

    o->initialized = true;
  }

  void setZero() {
    if (!residual_only_) {
      G.setZero();
    }
    AW.setZero();
    AQe.setZero();
    AQc.setZero();
    Ae.setZero();
    inner_product_of_w_and_c = 0;
    inner_product_of_c_and_e = 0;
    inner_product_of_c_and_Qc = 0;
    inner_product_of_c_and_Qe = 0;
  }

  friend void print(const WorkspaceSchurComplement& o) {
    DUMP(o.initialized);
    DUMP(o.AW);
    DUMP(o.AQc);
    DUMP(o.AQe);
    DUMP(o.Ae);
  }

  void InitializeWorkspace(double* data) { Initialize(this, data); }

  double inner_product_of_w_and_c;
  double inner_product_of_c_and_Qc;

  Eigen::Map<DenseMatrix, Eigen::Aligned> G{NULL, 0, 0};
  Eigen::Map<DenseMatrix, Eigen::Aligned> AW{NULL, 0, 0};
  Eigen::Map<DenseMatrix, Eigen::Aligned> AQc{NULL, 0, 0};
  // Self-dual embedding
  double inner_product_of_c_and_e;
  double inner_product_of_c_and_Qe;
  Eigen::Map<DenseMatrix, Eigen::Aligned> Ae{NULL, 0, 0};
  Eigen::Map<DenseMatrix, Eigen::Aligned> AQe{NULL, 0, 0};
  int m_;
  bool initialized = false;
  bool residual_only_ = false;
};

using SchurComplementSystem = WorkspaceSchurComplement;

}  // namespace conex
