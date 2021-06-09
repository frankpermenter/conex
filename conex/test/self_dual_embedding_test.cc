#include "conex/self_dual_embedding.h"

#include "conex/cone_program.h"
#include "conex/equality_constraint.h"
#include "conex/linear_constraint.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;

GTEST_TEST(Basic, Schur) {
  int n = 10;
  int m = 5;
  double wt = .9;
  VectorXd b = VectorXd::Random(m);
  VectorXd f = VectorXd::Random(m + 1);
  VectorXd e(n);
  e.setConstant(1);
  MatrixXd A = MatrixXd::Random(m, n);
  VectorXd c = VectorXd::Random(n);
  c = c.cwiseProduct(c);
  b = A * e;

  VectorXd w = e;
  Program prog(m);
  prog.AddConstraint(LinearConstraint(A.transpose(), c));
  prog.Initialize(SolverConfiguration());

  double sqrtmu = .1;
  for (int i = 0; i < 12; i++) {
    MatrixXd Qw = (w.cwiseProduct(w)).asDiagonal();
    MatrixXd Qwsqrt = w.asDiagonal();

//    SelfDualEmbeddingSystem sys(m);

    auto sys = prog.sys;
    prog.solver->Assemble();
    prog.solver->Factor();
    AssembleSchurComplement(&prog.kkt_system_manager_, &sys);
    

//    sys.AW = A * w;
//    sys.AQc = A * Qw * c;
//    sys.AQe = A * Qw * e;
//    sys.Ae = A * e;
//    sys.inner_product_of_c_and_e = c.transpose() * e;
//    sys.inner_product_of_c_and_w = c.transpose() * w;
//    sys.inner_product_of_c_and_Qc = c.transpose() * Qw * c;
//    sys.inner_product_of_c_and_Qe = c.dot(Qw * e);




    VectorXd Qc = Qw * c;
    double scale = Qc.norm() / c.norm();
    ;
    VectorXd cscale = c * scale;
    //sys.inner_product_of_c_and_Qc_scale = cscale.dot(Qc) / scale;

    //sys.AWA = A * Qw * A.transpose();

    auto sol = SolveEmbedding(sys, *prog.solver, b, wt, sqrtmu);

    // for (int i = 0; i < 3; i++) {
    //  sol += S.colPivHouseholderQr().solve(f - S * sol);
    //}

    // VectorXd y = sol.head(m);
    // VectorXd y = sol2;
    // dt = sol1(0);

    VectorXd y = sol.sol2;
    double dt = sol.sol1(0);

    double c_weight = wt * (1 + dt) - sqrtmu;
    double e_weight = sqrtmu;

    StepOptions options;
    StepInfo info;
    options.affine = 0;
    options.c_weight = c_weight;
    options.e_weight = 1;
    options.w_weight = e_weight;
    Ref ym(y.data(), y.rows(), 1);
    PrepareStep(&prog.kkt_system_manager_, options, ym, &info);
    options.step_size = 2.0 / (info.norminfd * info.norminfd);
    if (options.step_size > 1) {
      options.step_size = 1;
    }
    TakeStep(&prog.kkt_system_manager_, options);


    VectorXd slack = e_weight * e + c_weight * c - A.transpose() * y;
    VectorXd d = e - Qwsqrt * slack;
    DUMP(d);

    VectorXd errS = sqrtmu * Qwsqrt.inverse() * (e - d) -
                    (sqrtmu * wt * (1 + dt) * c + sqrtmu * sqrtmu * (e - c) -
                     A.transpose() * sqrtmu * y);

    VectorXd errX =
        A * (w + Qwsqrt * d) - (wt * (1 + dt) * b + sqrtmu * (A * e - b));

    double errG1 = b.dot(y) - c.dot(w + Qwsqrt * d);
    double errG2 = 1.0 / wt * (1 - dt) - sqrtmu * (c.transpose() * e + 1);

    double dinf = d.array().abs().maxCoeff();
    double stepsize = 2.0 / (dinf * dinf);

    std::cout << "\n tau: " << sqrtmu * wt << "  d:" << dinf
              << "  sqrtmu:" << sqrtmu << " , " << errX.norm() << " , "
              << errS.norm() << "," << errG1 - errG2;

    if (dinf < 1) {
      if (sqrtmu * sqrtmu < 1e-15) {
        double tau = sqrtmu * (wt * (1 + dt));
        VectorXd x = sqrtmu * (w + Qwsqrt * d) / tau;
        VectorXd s = sqrtmu * (w.cwiseInverse() - Qwsqrt.inverse() * d) / tau;
        DUMP(A * x - b);
        DUMP(x);
        DUMP(s);
        break;
      } else {
        //sqrtmu *= .1;
      }
    }

    if (stepsize > 1) {
      stepsize = 1;
    }
    d *= stepsize;
    VectorXd expd = d.array().exp();
    w = w.cwiseProduct(expd);
    wt = wt * std::exp(dt);
  }
}

}  // namespace conex
