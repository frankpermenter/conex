#include "conex/self_dual_embedding.h"

#include "conex/cone_program.h"
#include "conex/equality_constraint.h"
#include "conex/linear_constraint.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;

void BasicHSDSolverTestHelper(const Eigen::MatrixXd& A, 
                     const Eigen::VectorXd& bin, 
                     const Eigen::VectorXd& c,
                     int constraints_per_block = 3,
                     const Eigen::MatrixXd& B = MatrixXd(),
                     const Eigen::VectorXd& f = VectorXd()) {
  if (constraints_per_block == -1) {
    constraints_per_block = c.rows();
  } else {
    if ((A.cols() / constraints_per_block) * constraints_per_block != A.cols()) {
      EXPECT_TRUE(false);
      DUMP("Number of constraints must be divisible by constraints-per-block");
      return;
    }
  }

  int n = A.cols();
  int m = A.rows();

  VectorXd e(n);
  e.setConstant(1);

  VectorXd w = e;
  Program prog(m);
  int offset = 0;
  for (int i = 0; i < c.rows() / constraints_per_block; i++) {
    prog.AddConstraint(LinearConstraint(A.middleCols(offset, constraints_per_block).transpose(), c.segment(offset, constraints_per_block)));
    offset += constraints_per_block;
  }
  if (B.rows() > 0) {
    prog.AddConstraint(EqualityConstraints(B, f));
  }

  prog.Initialize(SolverConfiguration());
  VectorXd ysol; double kappa_sol; double tau_sol;
  SolveHSD(prog, bin, SolverConfiguration(), &ysol, &tau_sol, &kappa_sol);
  return;
}

void BasicTestHelper(const Eigen::MatrixXd& A, 
                     const Eigen::VectorXd& bin, 
                     const Eigen::VectorXd& c,
                     int constraints_per_block = 3,
                     const Eigen::MatrixXd& B = MatrixXd(),
                     const Eigen::VectorXd& f = VectorXd()) {

  if (constraints_per_block == -1) {
    constraints_per_block = c.rows();
  } else {
    if ((A.cols() / constraints_per_block) * constraints_per_block != A.cols()) {
      EXPECT_TRUE(false);
      DUMP("Number of constraints must be divisible by constraints-per-block");
      return;
    }
  }

  int n = A.cols();
  int m = A.rows();

  VectorXd e(n);
  e.setConstant(1);

  VectorXd w = e;
  Program prog(m);
  int offset = 0;
  for (int i = 0; i < c.rows() / constraints_per_block; i++) {
    prog.AddConstraint(LinearConstraint(A.middleCols(offset, constraints_per_block).transpose(), c.segment(offset, constraints_per_block)));
    offset += constraints_per_block;
  }
  if (B.rows() > 0) {
    prog.AddConstraint(EqualityConstraints(B, f));
  }

  prog.Initialize(SolverConfiguration());

  Eigen::VectorXd b(prog.kkt_system_manager_.SizeOfKKTSystem());
  b.setZero();
  b.head(m) << bin;


  auto sys = prog.sys;


  double sqrtmu = .1;
  double eps = 1e-6;
  // double wt = sqrtmu;
  double wt = .9;
  for (int i = 0; i < 25; i++) {
    MatrixXd Qw = (w.cwiseProduct(w)).asDiagonal();
    MatrixXd Qwsqrt = w.asDiagonal();


    prog.solver->Assemble();
    prog.solver->Factor();
    AssembleSchurComplement(&prog.kkt_system_manager_, &sys);
    EXPECT_NEAR((sys.AW.topRows(m) - A*w).norm() / (A*w).norm(), 0, eps);
    EXPECT_NEAR((sys.AQc.topRows(m) - A*Qw*c).norm() / (A*Qw*c).norm() , 0, eps);
    EXPECT_NEAR((sys.AQe.topRows(m) - A*Qw*e).norm() / (A*Qw*e).norm(), 0, eps);
    EXPECT_NEAR((sys.Ae.topRows(m) - A*e).norm()/(A*e).norm(), 0, eps);
    
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
    VectorXd lambda;
    if (B.rows() > 0) {
      lambda = y.bottomRows(B.rows());
    }

    double c_weight = wt * (1 + dt) - sqrtmu;
    double e_weight = sqrtmu;

    StepOptions options;
    StepInfo info;
    options.affine = 0;
    options.c_weight = c_weight;
    options.e_weight = 1;
    options.w_weight = e_weight;
    Ref ym(sol.sol2.data(), y.rows(), 1);
    PrepareStep(&prog.kkt_system_manager_, options, ym, &info);
    options.step_size = 2.0 / (info.norminfd * info.norminfd);
    if (options.step_size > 1) {
      options.step_size = 1;
    }
    TakeStep(&prog.kkt_system_manager_, options);


    VectorXd slack = e_weight * e + c_weight * c - A.transpose() * y.head(m);

    VectorXd d = e - Qwsqrt * slack;
    EXPECT_NEAR(d.squaredNorm(), info.normsqrd, eps * info.normsqrd);
    EXPECT_NEAR(d.array().abs().maxCoeff(), info.norminfd, eps);

    VectorXd errS = sqrtmu * Qwsqrt.inverse() * (e - d) -
                    (sqrtmu * wt * (1 + dt) * c + sqrtmu * sqrtmu * (e - c) -
                     A.transpose() * sqrtmu * y.head(m));

    VectorXd errX;
    if (B.rows() > 0) {
        errX = A * (w + Qwsqrt * d) + B.transpose() * y.tail(B.rows()) - (wt * (1 + dt) * b.head(m) + sqrtmu * (A * e - b.head(m)));
    } else {
        errX = A * (w + Qwsqrt * d) - (wt * (1 + dt) * b.head(m) + sqrtmu * (A * e - b.head(m)));
    }

    double errG1 = b.dot(y) - c.dot(w + Qwsqrt * d);
    double errG2 = 1.0 / wt * (1 - dt) - sqrtmu * (c.transpose() * e + 1);

    double dinf = d.array().abs().maxCoeff();
    double stepsize = 2.0 / (dinf * dinf);

    std::cout << "\n tau: " << sqrtmu * wt << "  d:" << dinf << "  dinf_step:" << info.norminfd
              << "  sqrtmu:" << sqrtmu << " , " << errX.norm() << " , "
              << errS.norm() << "," << errG1 - errG2;

    if (dinf < 1) {
      if (sqrtmu * sqrtmu < 1e-10) {
        double tau = sqrtmu * (wt * (1 + dt));
        VectorXd x = sqrtmu * (w + Qwsqrt * d) / tau;
        VectorXd s = sqrtmu * (w.cwiseInverse() - Qwsqrt.inverse() * d) / tau;
        if (B.cols() > 0) {
          EXPECT_NEAR((A * x + B.transpose() * y.tail(B.rows()) * sqrtmu/tau  - b.head(m)).norm(), 
                      0, eps); 
        } else {
          EXPECT_NEAR((A * x  - b.head(m)).norm(), 0, eps); 
        }
        EXPECT_NEAR(x.dot(s), 0, eps);
        if (B.rows() > 0) {
          EXPECT_NEAR((B * (y.head(m) * sqrtmu/tau) - f).norm(), 0, eps);
        }
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
    wt = wt * std::exp(stepsize*dt);
  }
}

struct TestData {
  MatrixXd A;
  VectorXd b;
  VectorXd c;
  MatrixXd B;
  VectorXd f;
};

TestData GetTestData() {
  srand(1);
  TestData d;
  int n = 10;
  int m = 5;
  double wt = .9;
  d.b = VectorXd::Random(m);
  VectorXd e(n);
  e.setConstant(1);
  d.A = MatrixXd::Random(m, n);
  d.c = VectorXd::Random(n);
  d.b = d.A * e;
  d.c = d.c.cwiseProduct(d.c);
  d.B.resize(1, m); d.B.setConstant(1);
  d.f.resize(1); d.f(0) = 1;
  return d;
}

GTEST_TEST(Basic, Schur1) {
  auto d = GetTestData();
  BasicTestHelper(d.A, d.b, d.c, 10, d.B, d.f);
}

GTEST_TEST(Basic, Schur2) {
  auto d = GetTestData();
  BasicTestHelper(d.A, d.b, d.c, 2, d.B, d.f);
}

/*
GTEST_TEST(Basic, Schur3) {
  auto d = GetTestData();
  BasicTestHelper(d.A, d.b, d.c, 5, d.B, d.f);
}*/

GTEST_TEST(Basic, Schur4) {
  auto d = GetTestData();
  BasicTestHelper(d.A, d.b, d.c, 10);
}
GTEST_TEST(Basic, Schur5) {
  auto d = GetTestData();
}
GTEST_TEST(Basic, Schur6) {
  auto d = GetTestData();
  BasicTestHelper(d.A, d.b, d.c, 2);
}
GTEST_TEST(Basic, Schur7) {
  auto d = GetTestData();
  BasicTestHelper(d.A, d.b, d.c, 5);
  BasicHSDSolverTestHelper(d.A, d.b, d.c, 5);
}



}  // namespace conex
