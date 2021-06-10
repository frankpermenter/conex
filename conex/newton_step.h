#pragma once
#include "debug_macros.h"
#include "memory_utils.h"
#include <Eigen/Dense>

namespace conex {

using DenseMatrix = Eigen::MatrixXd;
using Ref = Eigen::Map<DenseMatrix, Eigen::Aligned>;

/*
struct SelfDualEmbeddingSystem {
  SelfDualEmbeddingSystem(int m) : 
      AWA(m, m), AW(m, 1), AQc(m, 1), AQe(m, 1), Ae(m, 1) {}
  void setZero() { 
    AW.setZero();
    AQc.setZero();
    AQe.setZero();
    Ae.setZero();
    inner_product_of_c_and_e = 0;
    inner_product_of_c_and_w = 0;
    inner_product_of_c_and_Qc = 0;
    inner_product_of_c_and_Qe = 0;;
  };

  Eigen::MatrixXd AWA;
  Eigen::VectorXd AW;
  Eigen::VectorXd AQc;
  Eigen::VectorXd AQe;
  Eigen::VectorXd Ae;
  double inner_product_of_c_and_e;
  double inner_product_of_c_and_w;
  double inner_product_of_c_and_Qc;
  double inner_product_of_c_and_Qc_scale;
  double inner_product_of_c_and_Qe;
};*/

struct WeightedSlackEigenvalues {
  double limit = 0;
  double frobenius_norm_squared = 0;
  double trace = 0;
  double lambda_min = 1e30;
  double lambda_max = -1e30;
  double rank;
};

struct IterationStats {
  double norminf = 0;
};

struct StepOptions {
  bool affine = true;
  double inv_sqrt_mu = 0;
  // Take step of form  w_1 e + Q(w/2)(A^y - w_2 c)
  double c_weight = 0;
  double e_weight = 0;
  double w_weight = 0;
  double step_size = 1;
  double dinf_limit = 1;
};

struct SlackWeights {
  double c_weight = 0;
  double e_weight = 0;
};

using LineSearchParameters = StepOptions;
struct StepInfo {
  double normsqrd = 0;
  double norminfd = 0;
  double inv_sqrt_mu_primal_lower_bound = -std::numeric_limits<double>::max();
  double inv_sqrt_mu_primal_upper_bound = std::numeric_limits<double>::max();
  double inv_sqrt_mu_dual_lower_bound = -std::numeric_limits<double>::max();
  double inv_sqrt_mu_dual_upper_bound = std::numeric_limits<double>::max();
};

using DenseMatrix = Eigen::MatrixXd;
struct WorkspaceSchurComplement {
  WorkspaceSchurComplement(int m) : m_(m) {}
  WorkspaceSchurComplement() {}

  static constexpr int size_of(int m, bool residual_only) {
    if (residual_only) {
      return 4 * get_size_aligned(m);
    } else {
      return get_size_aligned(m * m) + 4 * get_size_aligned(m);
    }
  }

  friend int SizeOf(const WorkspaceSchurComplement& o) { return size_of(o.m_, o.residual_only_); }

  friend void Initialize(WorkspaceSchurComplement* o, double* data) {
    using Map = Eigen::Map<DenseMatrix, Eigen::Aligned>;
    int m = o->m_;
    new (&o->AW)
        Map(data, m, 1);
    new (&o->AQc)
        Map(data + 1 * get_size_aligned(m), m, 1);
    new (&o->AQe)
        Map(data + 2 * get_size_aligned(m), m, 1);
    new (&o->Ae)
        Map(data + 3 * get_size_aligned(m), m, 1);

    if (!o->residual_only_) {
      new (&o->G) Map(data + 4 * get_size_aligned(m), m, m);
    }

    o->initialized = true;
  }

  void setZero() {
    AW.setZero();
    AQe.setZero();
    AQc.setZero();
    Ae.setZero();
    inner_product_of_c_and_w = 0;
    inner_product_of_c_and_e = 0;;
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

  double inner_product_of_c_and_w;
  double inner_product_of_c_and_e;
  double inner_product_of_c_and_Qc;
  double inner_product_of_c_and_Qe;
  
  Eigen::Map<DenseMatrix, Eigen::Aligned> G{NULL, 0, 0};
  Eigen::Map<DenseMatrix, Eigen::Aligned> AW{NULL, 0, 0};
  Eigen::Map<DenseMatrix, Eigen::Aligned> AQc{NULL, 0, 0};
  Eigen::Map<DenseMatrix, Eigen::Aligned> AQe{NULL, 0, 0};
  Eigen::Map<DenseMatrix, Eigen::Aligned> Ae{NULL, 0, 0};
  int m_;
  bool initialized = false;
  bool residual_only_ = false;
};

using SchurComplementSystem = WorkspaceSchurComplement;
using SelfDualEmbeddingSystem = WorkspaceSchurComplement;

}  // namespace conex
