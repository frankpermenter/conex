#include "projection.h"
#include <bitset>
#include "conex/debug_macros.h"
#include <Eigen/Dense>
namespace conex {
namespace quadratic_programs {

using Eigen::MatrixXd;
using Eigen::VectorXd;

class CoordinateProjection {
 public:
  CoordinateProjection(const VectorXd& D) : D_(D) {
    is_identity = D.maxCoeff() < 1;
  }
  VectorXd Eval(const VectorXd& x) {
    VectorXd y = x;
    if (is_identity) {
      return x;
    }
    for (int i = 0; i < x.rows(); i++) {
      if (D_(i) < 1) {
        y(i) = 0;
      }
    }
    return y;
  }

  VectorXd OneMinusEval(const VectorXd& x) { return x - Eval(x); }

 private:
  VectorXd D_;
  bool is_identity = false;
};

class Ratios {
 public:
  Ratios(double sqrtmuinv, const VectorXd& exp_v, const VectorXd& l0,
         const VectorXd& s0) {
    s = 2 * 1.0 / sqrtmuinv * exp_v.cwiseInverse();
    l = 2 * 1.0 / sqrtmuinv * exp_v;
    r_s = (s0).cwiseProduct(s.cwiseInverse());
    r_l = l0.cwiseProduct(l.cwiseInverse());
  }

  VectorXd s;
  VectorXd l;
  VectorXd x;
  VectorXd r_s;
  VectorXd r_l;
};

MatrixXd GetRows(const MatrixXd& B, const std::vector<int>& indices) {
  MatrixXd B_submatrix(indices.size(), B.cols());
  int cnt = 0;
  for (const auto& i : indices) {
    B_submatrix.row(cnt++) = B.row(i);
  }
  return B_submatrix;
}

MatrixXd Contraction::EvalSubMatrix(const MatrixXd& X,
                                    const std::vector<int>& indices) {
  MatrixXd Z(indices.size(), X.cols());
  MatrixXd B_submatrix = GetRows(B_, indices);
  MatrixXd y = llt_.solve(B_submatrix.transpose() * X);
  return B_submatrix * y;
}

VectorXd GetIndicator(int n, const std::vector<int>& indices) {
  MatrixXd Z(n, 1);
  Z.setZero();
  for (const auto& i : indices) {
    Z(i) = 1;
  }
  return Z;
}

Direction NewtonDirectionFromProjection(const ProblemData& data,
                                        const VectorXd& exp_v,
                                        const double sqrtmuinv, double scale) {
  MatrixXd AD = exp_v.asDiagonal() * data.A;
  Contraction P(AD, data.W);
  std::vector<int> active_set;
  std::vector<int> inactive_set;
  for (int i = 0; i < exp_v.rows(); i++) {
    if (exp_v(i) >= 1) {
      active_set.push_back(i);
    } else {
      inactive_set.push_back(i);
    }
  }

  Direction d;
  d.d = exp_v;
  d.dlambda_times_d_slack = 1e4;

  if (active_set.size() > 0 && inactive_set.size() > 0) {
    Eigen::MatrixXd I_inact =
        Eigen::MatrixXd::Identity(inactive_set.size(), inactive_set.size());
    MatrixXd Pinact = P.EvalSubMatrix(I_inact, inactive_set);
    VectorXd s = 2 * 1.0 / sqrtmuinv * exp_v.cwiseInverse();
    VectorXd r_s = (data.b).cwiseProduct(s.cwiseInverse());
    VectorXd residual_s = GetRows(r_s, inactive_set);
    residual_s -= GetRows(P.Eval(r_s), inactive_set);
    Eigen::LLT<MatrixXd> llt_in(I_inact - Pinact);
    r_s = llt_in.solve(residual_s);

    MatrixXd Pact = P.EvalSubMatrix(
        Eigen::MatrixXd::Identity(active_set.size(), active_set.size()),
        active_set);
    Eigen::LLT<MatrixXd> llt(Pact);
    VectorXd l0 = data.A.transpose().colPivHouseholderQr().solve(data.c);
    VectorXd l = 2 * 1.0 / sqrtmuinv * exp_v;
    VectorXd r_l = (l0).cwiseProduct(l.cwiseInverse());
    VectorXd residual_l = GetRows(P.Eval(r_l), active_set);
    r_l = llt.solve(residual_l);
    double eps = 1e-4;
    if (r_l.minCoeff() >= -eps && r_l.maxCoeff() <= 1 + eps &&
        r_s.minCoeff() >= -eps && r_s.maxCoeff() <= 1 + eps) {
      d.dlambda_times_d_slack = 0;
      int cnt = 0;
      for (auto i : active_set) {
        d.d(i) = 2 * r_l(cnt++) - 1;
      }
      cnt = 0;
      for (auto i : inactive_set) {
        d.d(i) = 1 - 2 * r_s(cnt++);
      }
      DUMP(Pact);
      DUMP(r_l.minCoeff());
      DUMP(r_l.maxCoeff());
      DUMP(r_s.minCoeff());
      DUMP(r_s.maxCoeff());
    }
    int active_set_sum = 0;
    for (auto i : active_set) {
      active_set_sum += i;
    }
  }

  return d;
}

#if 0
Direction NewtonDirectionFromProjection(const ProblemData& data, 
                          const VectorXd& exp_v_in,
                          const double sqrtmuinv_in,
                          double scale) {

  const MatrixXd& W = data.W;
  const VectorXd& c = data.c;
  const MatrixXd& A = data.A;
  const VectorXd& b = data.b;

  VectorXd l0 = data.A.transpose().colPivHouseholderQr().solve(data.c);
  Ratios r0(sqrtmuinv_in, exp_v_in, l0, data.b);
  


  double sqrtmuinv = sqrtmuinv_in * scale;
  VectorXd exp_v = exp_v_in; 


  scale = 1;
  for (int i = 0; i < exp_v_in.rows(); i++) {
    if (exp_v_in(i) >= 1) {
      exp_v(i) *= scale;
    } else {
      exp_v(i) /= scale;
    }
  }


  MatrixXd AD = exp_v.asDiagonal() * data.A;
  Contraction P(AD, data.W);
  CoordinateProjection Pcoord(exp_v);


  Ratios r(sqrtmuinv, exp_v, l0, data.b);

  VectorXd e(exp_v.rows()); e.setConstant(1);
  VectorXd d_proj = e - 2 * r.r_s + 2 * Pcoord.Eval(r.r_l + r.r_s - e);

  VectorXd weighted_x0 = exp_v.asDiagonal().inverse() *  l0;
  VectorXd weighted_b = exp_v.asDiagonal()*  data.b;

  VectorXd project_coord =  2 * e - sqrtmuinv * (weighted_x0 + weighted_b); 

  VectorXd margin_l = Pcoord.Eval(P.Eval(r0.r_l)); 
  VectorXd error_l = P.Eval(r0.r_l) - Pcoord.Eval(P.Eval(r0.r_l));

  //DUMP(error_l.norm());
  //DUMP(margin_l);

  VectorXd margin_s = Pcoord.OneMinusEval(P.OneMinusEval(r0.r_s));
  VectorXd error_s = P.OneMinusEval(r0.r_s) - Pcoord.OneMinusEval(P.OneMinusEval(r0.r_s));

  // DUMP(error_s);
  // DUMP(error_l);
  // DUMP(error_s.norm());
  // DUMP(margin_l);
  // DUMP(margin_s);



  VectorXd dir_coord = -(Pcoord.Eval(project_coord) + sqrtmuinv * weighted_b);
  dir_coord.array() += 1;

  VectorXd dir_actual = -(P.Eval(project_coord) + sqrtmuinv * weighted_b);
  dir_actual.array() += 1;


  //DUMP(dir_coord);
  //DUMP(project_coord);
  //DUMP(Pcoord.Eval(project_coord));
  //DUMP(sqrtmuinv * weighted_b);
  //DUMP(sqrtmuinv_in * exp_v_in.cwiseProduct(data.b));
  //DUMP(data.b);
  //DUMP(1.0/sqrtmuinv * exp_v);

  //MatrixXd Pmatrix = P.Eval(MatrixXd::Identity(l0.rows(), l0.rows()));
  //DUMP(Pmatrix);
  //DUMP("HEHEHE");
  //DUMP(Pmatrix);
  //DUMP(Pmatrix*project_coord);
  //DUMP(Pcoord.Eval(project_coord));
  //DUMP(dir_coord);
  //DUMP(dir_actual);

  Direction d;
  d.d = dir_coord;
  d.dlambda_times_d_slack = std::sqrt(error_s.squaredNorm() + error_l.squaredNorm());
//  d.d = dir_actual;
  return d;
}
#endif

}  // namespace quadratic_programs
}  // namespace conex
