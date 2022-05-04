#define EIGEN_RUNTIME_NO_MALLOC 1
#define CONEX_ENABLE_TIMER 1
#include <map>
#include <memory>
#include <numeric>
#include <stack>

#include "conex/error_checking_macros.h"
#include "conex/RLDLT.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

#include <stdlib.h>

namespace conex {

class Workspace {
 public: 
  Eigen::MatrixXd separator_rows; 
  Eigen::MatrixXd separator_schur_complement;
  Eigen::MatrixXd supernode_submatrix;
  Eigen::MatrixXd temp;
  Eigen::LLT<Eigen::MatrixXd> llt;

  void Init(int m, int n) {
    separator_schur_complement.resize(n, n);
    temp.resize(m, n);
    supernode_submatrix.setIdentity(m, m);
    separator_rows.setRandom(n, m);
    llt.compute(supernode_submatrix);
  }
};

void ApproachFaster(Workspace* w) {
  w->separator_schur_complement.noalias() -= 
    w->separator_rows * w->llt.solve(w->separator_rows.transpose());
}
void ApproachSlower(Workspace* w) {
  w->temp.transpose().noalias() = w->llt.solve(w->separator_rows.transpose());
  w->separator_schur_complement.noalias() -= w->separator_rows * w->temp.transpose();
}

Workspace w; 
GTEST_TEST(LLTProfile, SchurComplementCalculation) {
  w.Init(10, 50);
  int num_trials = 200;
  double norm = 0;

#if 1
START_TIMER(ApproachFaster)
  for (int i = 0; i < num_trials; i++){
    ApproachFaster(&w);
    norm += w.separator_schur_complement(0, 0);
  }
END_TIMER
//#else
START_TIMER(ApproachSlower)
  for (int i = 0; i < num_trials; i++){
    ApproachSlower(&w);
    norm += w.separator_schur_complement(0, 0);
  }
END_TIMER
#endif
DUMP(norm);
Eigen::internal::set_is_malloc_allowed(true);
}

}  // namespace conex
