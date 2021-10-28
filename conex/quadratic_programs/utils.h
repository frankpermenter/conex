#include <Eigen/Dense>

#include "logspace_interior_point_method.h"

namespace conex {
namespace quadratic_programs {

struct Direction {
  Eigen::VectorXd d;
  Eigen::VectorXd x;
  double sqrtmu;
  double scale_c;
  double scale_b;
};


ProblemData RescaleProblemData(const ProblemData& data);
bool CheckPrimalInfeasibility(const ProblemData& data, 
                        const Variable& w, 
                        const Direction& d, double* descent,
                        Variable* certificate);

bool CheckDualInfeasibility(const ProblemData& data, 
                        const Variable& w, 
                        const Direction& d, double* descent,
                        Variable* certificate);


double UpperBound(const ProblemData& data, const Eigen::VectorXd& exp_v);

Direction NewtonDirection(const ProblemData& data, const Eigen::VectorXd& exp_v,
                          const double sqrtmuinv);

Direction PredictorDirection(const ProblemData& data,
                             const Eigen::VectorXd& exp_v);

Direction DualNewtonDirection(const ProblemData& data,
                              const Eigen::VectorXd& exp_v,
                              const Eigen::VectorXd& x, const double sqrtmuinv);

double Rescale(const ProblemData& data, double sqrtmu, const Variable& v);
double FindMinimumMu(const Eigen::VectorXd& d0, const Eigen::VectorXd& t,
                     double dinfmax);

int Solver(const ProblemData& data);

}  // namespace quadratic_programs
}  // namespace conex
