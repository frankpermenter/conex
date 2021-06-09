#include "conex/self_dual_embedding.h"

#include "conex/cone_program.h"
#include "conex/equality_constraint.h"
#include "conex/linear_constraint.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;

void BasicTestHelper(const Eigen::MatrixXd& A, 
                     const Eigen::VectorXd& b, 
                     const Eigen::VectorXd& c,
                     int constraints_per_block = 3) {
  if (constraints_per_block == -1) {
    constraints_per_block = c.rows();
  } else {
      DUMP(A.cols() / constraints_per_block);
    if ((A.cols() / constraints_per_block) * constraints_per_block != A.cols()) {
      EXPECT_TRUE(false);
      DUMP("Number of constraints must be divisible by constraints-per-block");
      return;
    }
  }

  int n = A.cols();
  int m = A.rows();
  double wt = .9;
  VectorXd e(n);
  e.setConstant(1);

  VectorXd w = e;
  Program prog(m);
  int offset = 0;
  for (int i = 0; i < c.rows() / constraints_per_block; i++) {
    prog.AddConstraint(LinearConstraint(A.middleCols(offset, constraints_per_block).transpose(), c.segment(offset, constraints_per_block)));
    offset += constraints_per_block;
  }

  prog.Initialize(SolverConfiguration());

  double sqrtmu = .1;
  double eps = 1e-6;
  for (int i = 0; i < 20; i++) {
    MatrixXd Qw = (w.cwiseProduct(w)).asDiagonal();
    MatrixXd Qwsqrt = w.asDiagonal();

//    SelfDualEmbeddingSystem sys(m);

    auto sys = prog.sys;
    prog.solver->Assemble();
    prog.solver->Factor();
    AssembleSchurComplement(&prog.kkt_system_manager_, &sys);
    EXPECT_NEAR((sys.AW - A*w).norm() / (A*w).norm(), 0, eps);
    EXPECT_NEAR((sys.AQc - A*Qw*c).norm() / (A*Qw*c).norm() , 0, eps);
    EXPECT_NEAR((sys.AQe - A*Qw*e).norm() / (A*Qw*e).norm(), 0, eps);
    EXPECT_NEAR((sys.Ae - A*e).norm()/(A*e).norm(), 0, eps);
    

//    sys.AW = A * w;
//    sys.AQc = A * Qw * c;
//    sys.AQe = A * Qw * e;
//    sys.Ae = A * e;
    EXPECT_NEAR(sys.inner_product_of_c_and_e, c.transpose() * e, eps * (c.transpose() * e).norm());
    EXPECT_NEAR(sys.inner_product_of_c_and_w, c.transpose() * w, eps * (c.transpose() * w).norm());
    EXPECT_NEAR(sys.inner_product_of_c_and_Qc, c.transpose() * Qw * c,   (c.transpose() * Qw * c).norm()*  eps);
    EXPECT_NEAR(sys.inner_product_of_c_and_Qe, c.dot(Qw * e),  std::abs(c.dot(Qw * e)) * eps);

    VectorXd Qc = Qw * c;
    double scale = Qc.norm() / c.norm();
    ;
    VectorXd cscale = c * scale;

    auto sol = SolveEmbedding(sys, *prog.solver, b, wt, sqrtmu);
    VectorXd y = sol.sol2; double dt = sol.sol1(0);

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
    EXPECT_NEAR(d.squaredNorm(), info.normsqrd, eps);
    EXPECT_NEAR(d.array().abs().maxCoeff(), info.norminfd, eps);

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
      if (sqrtmu * sqrtmu < 1e-10) {
        double tau = sqrtmu * (wt * (1 + dt));
        VectorXd x = sqrtmu * (w + Qwsqrt * d) / tau;
        VectorXd s = sqrtmu * (w.cwiseInverse() - Qwsqrt.inverse() * d) / tau;
        DUMP(A * x - b);
        DUMP(x);
        DUMP(s);
        break;
      } else {
        sqrtmu *= .01;
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

  b = A * e;
  c = c.cwiseProduct(c);

  BasicTestHelper(A, b, c, 1);
  BasicTestHelper(A, b, c, 2);
  BasicTestHelper(A, b, c, 5);
}



}  // namespace conex
