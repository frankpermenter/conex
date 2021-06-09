#include "conex/cone_program.h"
#include "conex/equality_constraint.h"
#include "conex/linear_constraint.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct SelfDualEmbeddingSystem {
  MatrixXd AWA;
  VectorXd AW;
  VectorXd AQc;
  VectorXd AQe;
  VectorXd Ae;
  double inner_product_of_c_and_e;
  double inner_product_of_c_and_w;
  double inner_product_of_c_and_Qc;
  double inner_product_of_c_and_Qc_scale;
  double inner_product_of_c_and_Qe;
};



VectorXd BuildRHS(SelfDualEmbeddingSystem& s,
                  const VectorXd& b,
                  const double& wt,
                  const double& sqrtmu) {
  int m = b.rows();
  VectorXd f(m+1);
  //f.head(m) = wt*(b+AQc) + sqrtmu * ( A* Qw * (e-c) + A*e-b) - 2*A*w;
  f.head(m) = wt*(b+s.AQc) + sqrtmu * (s.AQe-s.AQc + s.Ae-b) - 2*s.AW;
  //f(m) = 1.0/wt + 2*c.transpose() * w 
  //     - wt*c.transpose() * Qw * c - sqrtmu * c.transpose() * Qw*(e-c) - sqrtmu * (c.dot(e) + 1);
  //f(m) = 1.0/wt + 2*s.inner_product_of_c_and_w
  //     - wt*s.inner_product_of_c_and_Qc - sqrtmu * (s.inner_product_of_c_and_Qe  - s.inner_product_of_c_and_Qc) - sqrtmu * (s.inner_product_of_c_and_e + 1);




  //f(m) = 1.0/wt + 2*s.inner_product_of_c_and_w
  //     - wt*s.inner_product_of_c_and_Qc - sqrtmu * (s.inner_product_of_c_and_Qe - s.inner_product_of_c_and_Qc)
             //                           - sqrtmu * (s.inner_product_of_c_and_e + 1);
  f(m) =  1.0/wt + 2*s.inner_product_of_c_and_w - wt*( s.inner_product_of_c_and_Qc_scale)
            - sqrtmu * (  s.inner_product_of_c_and_Qe - s.inner_product_of_c_and_Qc_scale) 
            - sqrtmu * (s.inner_product_of_c_and_e + 1);


  return f;
}

struct SelfDualEmbeddingSolution {
  VectorXd sol1;
  VectorXd sol2;
};

SelfDualEmbeddingSolution SolveEmbedding(SelfDualEmbeddingSystem& s,
                  const VectorXd& b,
                  const double& wt,
                  const double& sqrtmu) {


  SelfDualEmbeddingSolution sol;
  int m = b.rows();
  auto f = BuildRHS(s, b, wt, sqrtmu);
//  auto f = rhs;

  const MatrixXd& S11 = s.AWA;
  const MatrixXd& S21 = b.transpose() - s.AQc.transpose();
  const MatrixXd& S12 = -wt*(s.AQc + b);
  MatrixXd S22(1, 1);
  S22(0, 0) = wt*s.inner_product_of_c_and_Qc + 1.0/wt; 

  Eigen::LLT<MatrixXd> LLT(S11);

  //  S11 S12
  //  S21 S22
  sol.sol1 = (S22 - S21 * LLT.solve(S12)).eval().inverse() * (f.tail(1) - 
                                        S21 * LLT.solve(f.head(m)));

  sol.sol2 =  LLT.solve(f.head(m)  - S12 * sol.sol1);
  return sol;
}



GTEST_TEST(Basic, Schur)  {
  int n = 10;
  int m = 5;
  double wt = .9;
  VectorXd b = VectorXd::Random(m);
  VectorXd f = VectorXd::Random(m + 1);
  VectorXd e(n); e .setConstant(1);
  MatrixXd A = MatrixXd::Random(m, n);
  VectorXd c = VectorXd::Random(n); c = c.cwiseProduct(c);
  b = A * e;

  VectorXd w = e;

  double sqrtmu = .1;
  for (int i = 0; i < 70; i++) {

    MatrixXd Qw = (w.cwiseProduct(w)).asDiagonal();
    MatrixXd Qwsqrt = w.asDiagonal();

    SelfDualEmbeddingSystem sys;
    sys.AW = A*w;
    sys.AQc = A*Qw*c;
    sys.AQe = A * Qw * e;
    sys.Ae = A * e;
    sys.inner_product_of_c_and_e = c.transpose() * e;
    sys.inner_product_of_c_and_w = c.transpose() * w;
    sys.inner_product_of_c_and_Qc = c.transpose()*Qw*c;
    sys.inner_product_of_c_and_Qe = c.dot(Qw*e);

    VectorXd Qc = Qw*c;
    double scale = Qc.norm()/ c.norm();;
    VectorXd cscale = c*scale;
    sys.inner_product_of_c_and_Qc_scale = cscale.dot(Qc) / scale;


    sys.AWA = A*Qw*A.transpose();

#if 0
    MatrixXd S(m+1, m+1);

    S.topLeftCorner(m, m) = A*Qw*A.transpose();
    MatrixXd AQc = A*Qw*c;

    S.bottomLeftCorner(1, m) = b.transpose() - AQc.transpose();

    //S(m, m) = wt*c.transpose()*Qw*c + 1.0/wt; 
    S(m, m) = wt*sys.inner_product_of_c_and_Qc + 1.0/wt; 

    S.topRightCorner(m, 1) = -wt*(AQc + b);

    f.head(m) = wt*(b+AQc) + sqrtmu * ( A* Qw * (e-c) + A*e-b) - 2*A*w;

    f(m) = 1.0/wt + 2*c.transpose() * w - wt*c.transpose() * Qw * c 
            -sqrtmu * c.transpose() * Qw*(e-c) 
            -sqrtmu * (c.dot(e) + 1);


    
    //f(m) = 1.0/wt + 2*sys.inner_product_of_c_and_w - wt*sys.inner_product_of_c_and_Qc 
    //          - sqrtmu * (sys.inner_product_of_c_and_Qe_minus_c) 
    //          - sqrtmu * (sys.inner_product_of_c_and_e + 1);

#if 0
    /*Works*/
    f(m) =  1.0/wt + 2*sys.inner_product_of_c_and_w - wt*c.transpose()*Qw*c
              - sqrtmu * (  sys.inner_product_of_c_and_Qe - sys.inner_product_of_c_and_Qc) 
              - sqrtmu * (sys.inner_product_of_c_and_e + 1);
#else
    /*Fails*/
    double c_dot_Qwc = cscale.dot(Qc) / scale;
    f(m) =  1.0/wt + 2*sys.inner_product_of_c_and_w - wt*( sys.inner_product_of_c_and_Qc_scale)
              - sqrtmu * (  sys.inner_product_of_c_and_Qe - sys.inner_product_of_c_and_Qc_scale) 
              - sqrtmu * (sys.inner_product_of_c_and_e + 1);
#endif

    auto f2 = BuildRHS(sys, b, wt, sqrtmu);




    double dt = 0;
    //VectorXd sol = S.colPivHouseholderQr().solve(f);

    // S11 S12
    // S21 S22
    //
    // S22 - S21 inv(S11) S12 = b2 - S21 inv(S11) b1

    MatrixXd S11 = A*Qw*A.transpose();
    MatrixXd S21 = S.bottomLeftCorner(1, m); 
    MatrixXd S22 = S.bottomRightCorner(1, 1);
    MatrixXd S12 = S.topRightCorner(m, 1);

    Eigen::LLT<MatrixXd> LLT(S11);
    if (LLT.info() != Eigen::Success) {
      break;
    }

    double schur_complement =  (S22 - S21 * LLT.solve(S12)).eval()(0, 0);
    if (std::fabs(schur_complement) <= 0) {
      DUMP("NOO!");
      return;
      schur_complement = 1e-15;
    }
    DUMP(schur_complement);
    VectorXd sol1 = 1.0/(schur_complement) * (f.tail(1) - 
                                          S21 * LLT.solve(f.head(m)));

    VectorXd sol2 =  LLT.solve(f.head(m)  - S12 * sol1);
#endif

    auto sol = SolveEmbedding(sys,  b, wt, sqrtmu);

    //for (int i = 0; i < 3; i++) {
    //  sol += S.colPivHouseholderQr().solve(f - S * sol);
    //}

    //VectorXd y = sol.head(m);
    //VectorXd y = sol2;
    //dt = sol1(0);

    VectorXd y = sol.sol2;
    double dt =sol.sol1(0);


    double c_weight = wt*(1+dt) - sqrtmu;
    double e_weight = sqrtmu; 
    VectorXd slack = e_weight * e + c_weight * c - A.transpose() * y;
    VectorXd d = e - Qwsqrt * slack;

    VectorXd errS = sqrtmu * Qwsqrt.inverse() * (e-d) - 
                     (  sqrtmu * wt*(1+dt)*c + sqrtmu*sqrtmu*(e-c) - A.transpose() * sqrtmu*y);


    VectorXd errX = A*(w+Qwsqrt*d) - ( wt*(1+dt)*b + sqrtmu * (A*e-b));

    double errG1 = b.dot(y) - c.dot (w+Qwsqrt*d);
    double errG2 =  1.0/wt*(1-dt)-sqrtmu*(c.transpose() * e + 1);

    double dinf = d.array().abs().maxCoeff();
    double stepsize =  2.0/(dinf * dinf);

    std::cout << "\n tau: " << sqrtmu*wt <<  "  d:" <<  dinf << "  sqrtmu:" << sqrtmu <<" , " << errX.norm() << " , " 
        << errS.norm() << "," << errG1-errG2;

    if (dinf < 1) {
      if (sqrtmu*sqrtmu < 1e-15) {
      double tau = sqrtmu * (wt*(1+dt));
      VectorXd x = sqrtmu * (w+Qwsqrt*d)/tau;
      VectorXd s = sqrtmu * (w.cwiseInverse()- Qwsqrt.inverse()*d)/tau;
      break;
      } else {
        sqrtmu *= .1;
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
