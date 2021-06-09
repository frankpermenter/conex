#include "conex/self_dual_embedding.h"
#include "conex/debug_macros.h"
namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace {
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

class KKTSolver {
 public:
  KKTSolver(const MatrixXd& S) : LLT_(S) {}
  void SolveInPlace(VectorXd * d) { LLT_.solveInPlace(*d); }
  VectorXd Solve(const VectorXd& d) const { 
    return LLT_.matrixL().transpose().solve(LLT_.matrixL().solve(d));
  }

  Eigen::LLT<MatrixXd> LLT_;
};

//template<typename T>
using T = KKTSolver;
SelfDualEmbeddingSolution SolveEmbeddingHelper(SelfDualEmbeddingSystem& s,
                  const T& kkt_solver,
                  const VectorXd& b,
                  const double& wt,
                  const double& sqrtmu) {
  static double schur_complement_system_last = -1;
  SelfDualEmbeddingSolution sol;
  int m = b.rows();
  auto f = BuildRHS(s, b, wt, sqrtmu);

  const MatrixXd& S11 = s.AWA;
  const MatrixXd& S21 = b.transpose() - s.AQc.transpose();
  const MatrixXd& S12 = -wt*(s.AQc + b);
  MatrixXd S22(1, 1);
  S22(0, 0) = wt*s.inner_product_of_c_and_Qc + 1.0/wt; 

  double schur_complement_system = S22(0, 0);
  schur_complement_system -= (S21 * kkt_solver.Solve(S12))(0, 0);

  if (schur_complement_system == 0) {
    schur_complement_system = schur_complement_system_last;
  } 
  schur_complement_system_last = schur_complement_system;

  sol.sol1 = 1.0/schur_complement_system * (f.tail(1) - 
                                        S21 * kkt_solver.Solve(f.head(m)));

  VectorXd ref = f.head(m)  - S12 * sol.sol1;
  sol.sol2 =  kkt_solver.Solve(ref);
  return sol;
}

} // namespace



SelfDualEmbeddingSolution SolveEmbedding(SelfDualEmbeddingSystem& s,
                  const VectorXd& b,
                  const double& wt,
                  const double& sqrtmu) {

  KKTSolver solver(s.AWA);
  return SolveEmbeddingHelper(s, solver, b, wt, sqrtmu);

}
} // namespace
