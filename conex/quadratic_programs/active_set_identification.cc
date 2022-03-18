#include <Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;
#include "conex/debug_macros.h"
int ActiveSet() {
  int num_constraints = 4;
  int num_vars = 4;
  int num_active = 2;
  VectorXd lambda(num_constraints); lambda.setZero();
  VectorXd s;
  VectorXd c;
  MatrixXd A(num_constraints, num_vars); A.setRandom();

  lambda.head(num_active).setRandom();
  c = A.transpose() * lambda;



  MatrixXd W(num_vars, num_vars);
  W.setIdentity();
  W.diagonal().head(num_active).setConstant(3);

  MatrixXd WA = W * A;
  MatrixXd G = WA.transpose() * WA;
  Eigen::LLT<MatrixXd> llt(G);
  MatrixXd P = WA * llt.solve((WA).transpose());


  // Guess active set and perform cg iterations.
  // Solve A.transpose() * lam = c.
  //
  //VectorXd l0 = A.transpose().colPivHouseholderQr().solve(c);
  //VectorXd r_l = W.inverse() * l0;
  //VectorXd residual_l = P * r_l;
  //
  // residual_l = WA (AWWA)^{-1} A' W * Winv * l0
  // residual_l = WA * inv(G) * c;                           
  VectorXd residual_l = WA * llt.solve(c);

  MatrixXd Wa = W.topLeftCorner(num_active, num_active);
  MatrixXd Ps = P.topLeftCorner(num_active, num_active);

  VectorXd r_l = Ps.inverse() * residual_l.head(num_active);

  DUMP(lambda);
  DUMP(Wa * r_l);

  // G = inv(AWWA)
  // S = W A G A' W
  // residual_l
  // residual_l * r_l
  // xhat = W^{-1} x_i = A G A' W
  // phat = W^{-1} p_i = A G A' W
  // rhat = W^{-1} r_i = W^{-1}(res_l - S x_i)
  //                   = W^{-1} res_l - A G A' W x_i
  //
  //                     W^{-1} res_l = A * G c
  //
  //                     W^{-1} S x_i = A inv (A WW A)^{-1} A' W * W xhat
  //
  //  rhat = A G^{-1}(c - A' W * W * xhat)
  //            
  //  xhat += \|r_k\|^2/(p_k S p_k) phat
  //
  //  phat = rhat + phat * \|r_{k+1}\|^2 /\|r_{k}\|
  //
  //  where \|r\|^2 = rhat' W W rhat
  //        \|p' S p\|^2 =  phat(W W  A inv (A WW A) A' W W)
  //
}

int main() {
  ActiveSet();
}
