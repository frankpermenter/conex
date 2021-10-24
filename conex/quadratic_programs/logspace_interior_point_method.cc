#include "conex/debug_macros.h"
#include "two_dimensional_lp.h"
#include "utils.h"

namespace conex {
namespace quadratic_programs {
namespace {
using Eigen::MatrixXd;
using Eigen::VectorXd;

Variable InitializeFromX0(const ProblemData& data, const VectorXd& x0) {
  Variable v;
  VectorXd s0 = data.A * x0 + data.b;
  for (int i = 0; i < s0.rows(); i++) {
    if (s0(i) < 1e-9) {
      s0(i) = 1e-4;
    }
  }
  VectorXd lambda = s0.cwiseInverse();

  VectorXd k1 = data.A.transpose() * lambda;
  VectorXd k2 = data.W * x0 + data.c;

  double mu = (1e-8 + k2.norm()) / (1e-8 + k1.norm());
  v.expv = sqrt(mu) * lambda;
  return v;
}

struct Components {
  VectorXd d0;
  VectorXd d1;
  VectorXd d2;
};

ProblemData ThetaData(const ProblemData& datain, double theta) {
  ProblemData data = datain;
  VectorXd e(data.A.rows());
  e.setConstant(1);
  data.b = (1 - theta) * datain.b + theta * e;
  VectorXd chat = data.A.transpose() * e;
  data.c = (1 - theta) * datain.c + theta * chat;
  return data;
}

Variable LogspaceIPMHelper(const ProblemData& data_raw,
                           const SolverOptions& options,
                           const Variable& initial_point) {
  auto data = data_raw;

  MatrixXd G = data.A.transpose() * data.A;
  Eigen::LLT<MatrixXd> llt(G);
  MatrixXd P = data.A * llt.solve(data.A.transpose());
  VectorXd b0 = data.b - P * data.b;
  VectorXd l0 = data.A * llt.solve(data.c);

  auto datain = data_raw;

  int n = data.A.rows();
  if (b0.squaredNorm() > 1.0 / n) {
    double scale = 1.0 / std::sqrt(n) / b0.norm();
    datain.A *= scale;
    datain.b *= b0.norm();
    b0 *= scale;
  }

  if (l0.squaredNorm() > 1.0 / n) {
    double scale = 1.0 / std::sqrt(n) / l0.norm();
    datain.c *= scale;
    l0 *= scale;
  }

  bool adjust_theta = options.enable_dynamic_regularization;

  int m = data.A.rows();
  int warmstart_used = false;
  Variable v;
  if (initial_point.expv.size() > 0) {
    v = initial_point;
    warmstart_used = true;
    v.x.resize(data.A.cols());
  } else {
    v.expv.resize(m);
    v.expv.setConstant(1);
    v.x.resize(data.A.cols());
    v.x.setZero();
  }

  VectorXd e(m);
  e.setConstant(1);
  VectorXd lambda(m);
  VectorXd slack(m);
  double sqrtmu = 1;
  double mu_from_gap = 0;
  bool reset_mu = true;
  double k0 = 0;
  double k1 = 1;

  double theta = 1;  // sqrtmu;
  bool quit = false;
  for (int i = 0; i < options.maximum_iterations; i++) {
    double k = 1;

    VectorXd expmv = v.expv.cwiseInverse();
    VectorXd w = v.expv;

    Direction dir0;
    Direction dir1;
    if (reset_mu) {
      k0 = 0;
      k1 = 1;
    } else {
      reset_mu = false;
    }

    // DUMP(d0_.d + theta/sqrtmu * (dtheta_.d - dmu_.d) + 1.0/sqrtmu* (dmu_.d -
    // d0_.d)  - d);
    if (adjust_theta) {
      auto d0_ = NewtonDirection(ThetaData(datain, 1), v.expv, 0);
      auto dmu_ = NewtonDirection(ThetaData(datain, 0), v.expv, 1);
      auto dtheta_ = NewtonDirection(ThetaData(datain, 1), v.expv, 1);

      {
        VectorXd d0 = d0_.d;
        VectorXd d1 = dmu_.d - d0_.d;
        VectorXd d2 = dtheta_.d - dmu_.d;
        Limits lim;
        lim.theta_times_inv_sqrt_mu_ub = theta * sqrtmu;
        Limits theta_config;
        theta_config.theta_weight = options.theta_weight;
        theta_config.inv_sqrt_mu_weight = options.inv_sqrt_mu_weight;
        VectorXd extreme_points = InfinityNorm(d0, d1, d2, 1, theta_config);
        if (extreme_points.size() > 0) {
          sqrtmu = 1.0 / extreme_points(0);
          theta = extreme_points(1) * sqrtmu;
        } else {
          quit = true;
        }
        // throw std::runtime_error("DFDF");
      }
    } else {
      theta = 0;
    }

    data = ThetaData(datain, theta);
    // data.b = (1-theta) * datain.b + theta * e;
    // VectorXd chat = data.A.transpose() * e - data.W*v.x;
    // data.c = (1-theta) * datain.c + theta * chat;

    double w_winv;
#if 0
    {
      // VectorXd winv = w.cwiseInverse();
      VectorXd winv = slack / sqrtmu;
      VectorXd w = lambda / sqrtmu;
      double k = 1.0/sqrtmu;
      double kp = theta/sqrtmu;
      double g = l0.dot(winv) +  b0.dot(w);
      double ghat = (e-l0).dot(winv) +  (e-b0).dot(w);
      // DUMP(P * ( k * l0 + kp * (e - l0) - w));
      // DUMP(P * ( k * b0 + kp * (e - b0) - winv));
      // DUMP(k * b0 + kp * (e - b0) - winv);

      VectorXd s0 = b0;
      VectorXd x0 = l0;
      double rp = w.dot(k * s0 + kp * (e-s0) - winv);
      double rd = winv.dot(k * x0 + kp * (e-x0) - w);
      // DUMP(k*g + kp*ghat - (rp+rd));
      // DUMP(rp+rd);
      // DUMP( 2*k*kp * (s0+x0).dot(e) - k * g - kp * ghat + 2*kp*kp*(e-s0).dot(e-x0));
      // DUMP( 2*k*kp * (s0+x0).dot(e) - 2*(k * g + kp * ghat) + 2*kp*kp*(e-s0).dot(e-x0));
     
      w_winv = k * g + kp * ghat - (k*kp * (s0+x0).dot(e)) - kp*kp*(e-s0).dot(e-x0);
      DUMP(g);
      DUMP(x0.dot(s0));

    }
#endif

    dir0 = NewtonDirection(data, v.expv, 0);
    dir1 = NewtonDirection(data, v.expv, 1);

    VectorXd z = dir1.d - dir0.d;

    double mu_ls = -z.dot(dir0.d) / dir0.d.squaredNorm();

    if (!adjust_theta) {
      double dinfbound = options.dinf_limit;
      double upper_bound = FindMinimumMu(dir0.d, z, dinfbound);
      while (upper_bound < 0) {
        dinfbound *= 1.1;
        upper_bound = FindMinimumMu(dir0.d, z, dinfbound);
      }
      sqrtmu = 1.0 / upper_bound;
    }

    v.x = sqrtmu * (dir0.x + 1.0 / sqrtmu * (dir1.x - dir0.x));
    VectorXd d = dir0.d + 1.0 / sqrtmu * (dir1.d - dir0.d);

    // G x = k + k
    // DUMP(d0_.d + theta/sqrtmu * (dtheta_.d - dmu_.d) + 1.0/sqrtmu* (dmu_.d -
    // d0_.d)  - d);

    if (options.enable_dual_correction && i == options.maximum_iterations - 1) {
      auto dirP = DualNewtonDirection(data, v.expv, v.x, 1.0 / sqrtmu);
      double alpha = 1;
      v.x += alpha * sqrtmu * dirP.x;

      VectorXd dd = d + alpha * (dirP.d - d);
      lambda = sqrtmu * (v.expv + v.expv.cwiseProduct(dd));
    } else {
      lambda = sqrtmu * (v.expv + v.expv.cwiseProduct(d));
    }
    v.lambda = lambda;

    auto s = sqrtmu * (expmv - expmv.cwiseProduct(d));
    double gap = s.dot(lambda);

    double dfeas =
        (data.A.transpose() * lambda - (data.W * v.x + data.c)).norm();
    double dinf = d.array().abs().maxCoeff();
    double dnorm = d.norm();
    double stepsize = 2.0 / (dinf * dinf);

    mu_from_gap = sqrtmu * sqrtmu * (d.rows() - d.squaredNorm()) / d.rows();

    if (stepsize < 1) {
      d = d * stepsize;
    } else {
      stepsize = 1;
    }

    VectorXd expd = d.array().exp();
    v.expv = v.expv.cwiseProduct(expd);

    if (options.enable_rescaling) {
      k = Rescale(data, sqrtmu, v);
      v.expv.array() *= k;
    }

    std::cout << std::setprecision(2);
    std::cout << std::scientific;
    std::cout
        // << "Scaling: " << k
        << "  mu: " << sqrtmu * sqrtmu
        << "  gap_p: " << l0.dot(s) + b0.dot(lambda) << "  theta " << theta
        << "  gap:" << gap << "  w_dot_winv: "
        << w_winv
        //<< "  mu_from_gap" <<  mu_from_gap
        //        << " ipd0dt:  " << dir0.d.squaredNorm() - dir1.d.dot(dir0.d)
        //        << "  d0d1: " << dir0.d.squaredNorm() - dir1.d.squaredNorm()
        << " |d|_inf " << dinf << "  |d|^2 " << dnorm * dnorm
        << "  stepsize: " << stepsize << " gradx "
        << dfeas
        // << "  mu_ls " <<  mu_ls
        //  << "  min(s) " << (data.A*v.x + data.b).minCoeff()
        //  << "  min(lam) " << (lambda).minCoeff() <<
        //        "  all " <<  all_agree <<
        << "\n";

    slack = data.A * v.x + data.b;

    mu_from_gap = sqrtmu * sqrtmu * (d.rows() - d.squaredNorm()) / d.rows();

    if (quit) {
      DUMP("UNKNOWN ERROR");
      break;
    }
  }

  return v;
}
}  // namespace

Variable LogspaceIPM(const ProblemData& data, const SolverOptions& options,
                     const VectorXd& x0) {
  Variable v;
  if (x0.size() != 0) {
    v = InitializeFromX0(data, x0);
    v.x = x0;
  }
  return LogspaceIPM(data, options, v);
}

Variable LogspaceIPM(const ProblemData& data_raw, const SolverOptions& options,
                     const Variable& initial_point) {
  bool remove_equations = data_raw.B.rows() > 0;

  std::cout << "Starting the Conex optimizer...";
  std::cout << "\n Number of variables:  " << data_raw.W.cols();
  std::cout << "\n Number of inequalities: " << data_raw.A.rows();
  std::cout << "\n Number of equations: " << data_raw.B.rows();
  ;

  if (data_raw.A.rows() == 0) {
    std::cout << "\n Algorithm: analytical solution.";
    int num_eq = data_raw.B.rows();
    int size_kkt = data_raw.W.cols() + data_raw.B.rows();
    MatrixXd S(size_kkt, size_kkt);
    VectorXd f(S.rows());
    if (data_raw.B.rows() > 0) {
      S << data_raw.W, data_raw.B.transpose(), data_raw.B,
          Eigen::MatrixXd::Zero(num_eq, num_eq);
      f << -data_raw.c, data_raw.d;
    } else {
      S << data_raw.W;
      f << -data_raw.c;
    }

    Eigen::LDLT<Eigen::MatrixXd> llt(S);
    Variable sol;
    sol.x = llt.solve(f);
    sol.x = sol.x.topRows(data_raw.W.cols());
    return sol;
  } else {
    std::cout << "\n Algorithm: Logspace IPM";
  }
  std::cout << std::endl;

  //  Solve Bx = d as x0 + Bz where B is nullspace for B.
  //  W -> B'W B
  //  c' -> c' B + x0' W B
  //  A -> AB
  //  b -> b + Ax0
  //  B -> 0
  //  d -> 0;
  ProblemData data = data_raw;
  MatrixXd B_null_space;
  VectorXd x0;
  if (data.B.rows() > 0) {
    Eigen::FullPivLU<MatrixXd> lu(data_raw.B);
    B_null_space = lu.kernel();
    x0 = lu.solve(data.d);
    data.A = data_raw.A * B_null_space;
    data.c = B_null_space.transpose() * data_raw.c +
             B_null_space.transpose() * data_raw.W * x0;
    data.W = B_null_space.transpose() * data_raw.W * B_null_space;
    data.b += data_raw.A * x0;
  }
  auto solution = LogspaceIPMHelper(data, options, initial_point);
  if (remove_equations) {
    solution.x = B_null_space * solution.x + x0;
  }
  return solution;
}

}  // namespace quadratic_programs
}  // namespace conex
