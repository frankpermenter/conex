#pragma once
#include "conex/conex.h"
namespace conex {

inline SolverConfiguration DefaultTestConfiguration() {
  SolverConfiguration config;
#ifdef CONEX_TEST_CG
  config.kkt_solver = CONEX_KKT_SOLVER_CG;
#endif
  return config;
}

inline double DefaultEqualityConstraintTolerance() {
  double tolerance = 1e-5;
#ifdef CONEX_TEST_CG
  tolerance = 1e-4;
#endif
  return tolerance;
}

}  // namespace conex
