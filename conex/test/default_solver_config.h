#pragma once
#include "conex/cone_program.h"
namespace conex {

inline SolverConfiguration DefaultTestConfiguration() {
  SolverConfiguration config;
#ifdef CONEX_TEST_SUPERNODAL
  config.kkt_solver = CONEX_KKT_SOLVER_SUPERNODAL;
#endif
#ifdef CONEX_TEST_CG
  config.kkt_solver = CONEX_KKT_SOLVER_CG;
#endif
#ifdef CONEX_TEST_TREE
  config.kkt_solver = CONEX_KKT_SOLVER_TREE;
#endif

#ifdef CONEX_TEST_TREE
#ifdef CONEX_TEST_CG
#error("Cannot set both CONEX_TEST_TREE and CONEX_TEST_CG");
#endif
#endif

#ifdef CONEX_TEST_SUPERNODAL
#ifdef CONEX_TEST_CG
#error("Cannot set both CONEX_TEST_SUPERNODAL and CONEX_TEST_CG");
#endif
#endif

#ifdef CONEX_TEST_SUPERNODAL
#ifdef CONEX_TEST_TREE
#error("Cannot set both CONEX_TEST_SUPERNODAL and CONEX_TEST_TREE");
#endif
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
