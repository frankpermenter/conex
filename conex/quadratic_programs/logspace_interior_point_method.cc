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


struct Errors {
  double gap;
  double dfeas;
};

Errors ComputeErrors(const ProblemData& data_theta, 
                     const Variable& v,
                     const Direction& dir) {
    
  const auto& d = dir.d;
  double sqrtmu = dir.sqrtmu;
  VectorXd lambda = sqrtmu * (v.expv + v.expv.cwiseProduct(d));
  VectorXd expmv = v.expv.cwiseInverse();

  auto s = sqrtmu * (expmv - expmv.cwiseProduct(d));
  Errors error;
  error.gap = std::abs(s.dot(lambda));

  error.dfeas = (data_theta.A.transpose() * lambda - (data_theta.W * v.x + data_theta.c)).norm();
  return error;
}




Solution LogspaceIPMHelper(const ProblemData& data_input,
                           const SolverOptions& options,
                           const Variable& initial_point) {
  ProblemData data_rescaled = data_input;
  if (options.enable_rescaling) {
    std::cout << " Rescaling enabled.\n";
    data_rescaled = RescaleProblemData(data_input);
  } 
  

  bool adjust_theta = options.enable_dynamic_regularization;

  int m = data_rescaled.A.rows();
  int warmstart_used = false;
  Variable v;
  Variable primal_infeasiblity_certificate;
  Variable dual_infeasiblity_certificate;
  if (initial_point.expv.size() > 0) {
    v = initial_point;
    warmstart_used = true;
    v.x.resize(data_rescaled.A.cols());
  } else {
    v.expv.resize(m);
    v.expv.setConstant(1);
    v.x.resize(data_rescaled.A.cols());
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
  int status = CONEX_LOGSPACE_IPM_UNKNOWN;
  int i = 0;
  bool target_gap_reached = false;
  for (; i < options.maximum_iterations; i++) {
    adjust_theta = adjust_theta && theta > 1e-15;
    double k = 1;

    VectorXd w = v.expv;

    Direction dir0;
    Direction dir1;
    if (reset_mu) {
      k0 = 0;
      k1 = 1;
    } else {
      reset_mu = false;
    }

    dir0 = NewtonDirection(ThetaData(data_rescaled, 1), v.expv, 0);
    VectorXd d0 = dir0.d;
    double primal_ray_deriv; 
    double dual_ray_deriv; 
    bool primal_infeas = CheckPrimalInfeasibility(data_rescaled, v, dir0, &dual_ray_deriv, 
                                             &primal_infeasiblity_certificate);
    bool dual_infeas = CheckDualInfeasibility(data_rescaled, v, dir0, &primal_ray_deriv, 
                                         &dual_infeasiblity_certificate);

    double dinf_limit = options.dinf_limit;
    if ((target_gap_reached || adjust_theta) && dinf_limit   > 1) {
        dinf_limit = 1;
    }


    if (adjust_theta) {
      auto dmu_ = NewtonDirection(ThetaData(data_rescaled, 0), v.expv, 1);
      auto dtheta_ = NewtonDirection(ThetaData(data_rescaled, 1), v.expv, 1);
      {
        VectorXd d1 = dmu_.d - d0;
        VectorXd d2 = dtheta_.d - dmu_.d;
        Limits lim;

        bool no_feasible_theta = true;
        {
          Limits theta_config;
#if 0
          theta_config.inv_sqrt_mu_ub = 1e6;
          theta_config.inv_sqrt_mu_lb = 1e-4;
          theta_config.theta_times_inv_sqrt_mu_ub = 1e-4;
          theta_config.theta_times_inv_sqrt_mu_lb = -1e-9;
          theta_config.theta_weight = 1; 
          theta_config.inv_sqrt_mu_weight = -1; 
          VectorXd extreme_points = InfinityNorm(d0, d1, d2, options.dinf_limit, theta_config);
          if (extreme_points.size() > 0) {
            no_feasible_theta = false;
            adjust_theta = false;
            theta = 0;
          }
#else

          theta_config.theta_weight = options.theta_weight;
          theta_config.inv_sqrt_mu_weight = options.sqrt_mu_weight;
          auto vals = MuThetaSelect(d0, d1, d2, dinf_limit, theta_config);
          if (vals.success) {
            sqrtmu = vals.sqrtmu;
            if (vals.theta < 1e-9) {
              no_feasible_theta = false;
              adjust_theta = false;
              theta = 0;
            } else {
              theta = vals.theta;
            }
          }
#endif
        }


      }
        // throw std::runtime_error("DFDF");
    } else {
      theta = 0;
    }

    //theta = 1;
    //sqrtmu = 1;
    const ProblemData data_theta = ThetaData(data_rescaled, theta);
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

    dir1 = NewtonDirection(data_theta, v.expv, 1);
    VectorXd z = dir1.d - dir0.d;

    double mu_ls = -z.dot(dir0.d) / dir0.d.squaredNorm();

    double dinfbound = dinf_limit;
    if (!adjust_theta) {
      double upper_bound = FindMinimumMu(dir0.d, z, dinfbound);
      while (upper_bound < 0) {
        dinfbound *= 1.1;
        upper_bound = FindMinimumMu(dir0.d, z, dinfbound);
      }
      double sqrtmu_next = 1.0 / (1e-12 + upper_bound);
      if (sqrtmu_next > 1e4) {
        sqrtmu_next = 1e4;
      }
      if (sqrtmu_next < sqrtmu || i == 0) {
        sqrtmu = sqrtmu_next;
      }
    }

    if (sqrtmu * sqrtmu < options.minimum_mu) {
      sqrtmu = std::sqrt(options.minimum_mu);
    }


    Errors error;
    Direction dstep;
    dstep.sqrtmu = sqrtmu;
    dstep.x = sqrtmu * (dir0.x + 1.0 / sqrtmu * (dir1.x - dir0.x));
    dstep.d = dir0.d + 1.0 / sqrtmu * (dir1.d - dir0.d);
    v.x = dstep.x;
    error = ComputeErrors(data_theta, v, dstep);

    //double feasible_sqrtmu = FindMinimumMu(dir0.d, z, 1.0);
    //if (feasible_sqrtmu > 0) {
    //  Direction d_feas;
    //  d_feas.d = dir0.d + 1.0 / feasible_sqrtmu * (dir1.d - dir0.d);
    //  d_feas.x = dir0.x + 1.0 / feasible_sqrtmu * (dir1.x - dir0.x);
    //  d_feas.sqrtmu = sqrtmu;
    //  error = ComputeErrors(data_theta, v, d_feas);
    //  v.x = d_feas.x;  
    //} else {
    //  error = ComputeErrors(data_theta, v, dstep);
    //}


    auto& d = dstep.d;
    if (options.enable_dual_correction && i == options.maximum_iterations - 1) {
      auto dirP = DualNewtonDirection(data_theta, v.expv, v.x, 1.0 / sqrtmu);
      double alpha = 1;
      v.x += alpha * sqrtmu * dirP.x;

      VectorXd dd = d + alpha * (dirP.d - d);
      lambda = sqrtmu * (v.expv + v.expv.cwiseProduct(dd));
    } else {
      lambda = sqrtmu * (v.expv + v.expv.cwiseProduct(d));
    }
    v.lambda = lambda;


    double dinf = d.array().abs().maxCoeff();
    double dnorm = d.norm();
    double stepsize = 2.0 / (dinf * dinf);
    if (stepsize > 1) {
      stepsize = 1;
    }



    // Goal: rescale v and mu such that mu e^{v}(1+d) and mu e^{-v}(1-d)
    // don't change "much". We can scale up v such that e^{-v} absorbs
    // (1-d). So the scaling is alpha = log(1-d)
    VectorXd scaling(d.rows()); scaling.setConstant(1);
    scaling.array() -= d.array().abs();
    double mean_scaling = scaling.mean();
    scaling.array() -= mean_scaling;
    double scaling_dev = scaling.array().abs().maxCoeff();
    bool enable_rapid_mu = (scaling_dev < .15 || sqrtmu < 1e-9) && (dinf < 1.01);







    bool quadratic_convergence = false;
    if (enable_rapid_mu) {
      if (!adjust_theta) {
        quadratic_convergence = true;
        int n = v.expv.rows();
        double scale = mean_scaling * 1.0/10; 
        for (int i = 0; i < n; i++) {
          if (v.expv(i) > 1) {
            v.expv(i) *= (2-scale);
          } else {
            v.expv(i) /= (2-scale);
          }
        }
      }
    }

    //if (quadratic_convergence) {
    //  stepsize *= 2;
    //}

    if (stepsize != 1) {
      d = d * stepsize;
    }

      VectorXd expd = d.array().exp();
      v.expv = v.expv.cwiseProduct(expd);


    //  k e^a = e^a (1+d1)
    //  k e^{-b} = e^{-b} (1-d2)
    //
    //  k e^{b} = e^{-b} (1+d2)
    //  k e^{-a} = e^{-a} (1-d1) 

    //  k e^a = e^a .1
    //  k e^{-b} = e^{-b} .1 
    //
    //  k e^{b} = e^{-b} 1.9
    //  k e^{-a} = e^{-a} 1.9 

    //if (options.enable_rescaling) {
    //  k = Rescale(data_theta, sqrtmu, v);
    //  v.expv.array() *= k;
    //}

    std::cout << std::setprecision(4);
    std::cout << std::scientific;
    if (adjust_theta) {
    std::cout << "  ";
    } else {
    std::cout << "* ";
    }
    if (quadratic_convergence) {
    std::cout << "- ";
    } else {
    std::cout << "  ";
    }

    if (i < 10) {
      std::cout << i  << " ";
    } else {
      std::cout << i;
    }
    std::cout
        << "  mu: " << sqrtmu * sqrtmu
        //<< "  gap_p: " << l0.dot(s) + b0.dot(lambda) 
        << "  theta " << theta
        << "  gap:" << error.gap 
        //<< "  mu_from_gap" <<  mu_from_gap
        //        << " ipd0dt:  " << dir0.d.squaredNorm() - dir1.d.dot(dir0.d)
        //        << "  d0d1: " << dir0.d.squaredNorm() - dir1.d.squaredNorm()
        << " |d|_inf " << dinf 
        //<< "  |d|^2 " << dnorm * dnorm
        //<< "  stepsize: " << stepsize 
        << " scaling " << mean_scaling
        << " scaling dev" << scaling_dev
        << " dual_ray " << dual_ray_deriv 
        << " primal_ray " << primal_ray_deriv 
//        << " p_scale " << dir.scale_c
//        << " d_scale " << dir.scale_b

 //       << " gradx " << error.dfeas
        // << "  mu_ls " <<  mu_ls
        //  << "  min(s) " << (data.A*v.x + data.b).minCoeff()
        //  << "  min(lam) " << (lambda).minCoeff() <<
        //        "  all " <<  all_agree <<
        << "\n";

    slack = data_theta.A * v.x + data_theta.b;

    if (error.gap <= options.target_duality_gap) {
      target_gap_reached = true;
    }

    mu_from_gap = sqrtmu * sqrtmu * (d.rows() - d.squaredNorm()) / d.rows();
    if (error.gap <= options.target_duality_gap && dinf <= (1 + 1e-2) && 
        theta <= options.theta_truncation_threshold) {
      std::cout << "\n Primal-dual optimal solutions found." << std::endl;
      status = CONEX_LOGSPACE_IPM_SOLVED;
      break;
    }

    if (quit) {
      std::cout << "\n Numerical errors encountered. Terminating" << std::endl;
      break;
    }

    if (primal_infeas) { 
      std::cout << "\nThe primal is infeasible.";
    }
    if (dual_infeas) { 
      std::cout << "\nThe dual is infeasible.";
    }
    if (dual_infeas || primal_infeas) {
      status = CONEX_LOGSPACE_IPM_INFEASIBLE;
      std::cout << std::endl;
      break;
    }
  }

  Solution sol;

  sol.status = status;
  sol.iterations = i;
  switch (status) {
    case CONEX_LOGSPACE_IPM_SOLVED:
      sol.x = v;
      break;
    case CONEX_LOGSPACE_IPM_INFEASIBLE:
      sol.x = primal_infeasiblity_certificate; 
      sol.status = CONEX_LOGSPACE_IPM_INFEASIBLE;
      break;
    default:
      sol.x = v; sol.x.x.setConstant(std::sqrt(-1));
      sol.status = CONEX_LOGSPACE_IPM_UNKNOWN;
  }
  return sol;
}
}  // namespace

Solution LogspaceIPM(const ProblemData& data, const SolverOptions& options,
                     const VectorXd& x0) {
  Variable v;
  if (x0.size() != 0) {
    v = InitializeFromX0(data, x0);
    v.x = x0;
  }
  return LogspaceIPM(data, options, v);
}

Solution LogspaceIPM(const ProblemData& data_raw, const SolverOptions& options,
                     const Variable& initial_point) {
  bool remove_equations = data_raw.B.rows() > 0;

  std::cout << "Starting the Conex optimizer...";
  std::cout << "\n Number of variables:  " << data_raw.W.cols();
  std::cout << "\n Number of inequalities: " << data_raw.A.rows();
  std::cout << "\n Number of equations: " << data_raw.B.rows();
  
  // Minimize x' W x + c'x 
  // Bx = d
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

    Eigen::FullPivLU<Eigen::MatrixXd> llt(S);
    Solution sol;
    sol.x.x = llt.solve(f).topRows(data_raw.W.cols());
    sol.status = CONEX_LOGSPACE_IPM_SOLVED;
    sol.iterations = 0;
    std::cout << std::endl;
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
    // if (lu.info() != Eigen::Success) {
    //   std::cout << "\n Presolve failed. Aborting...";
    //   Solution sol;
    //   sol.status = CONEX_LOGSPACE_IPM_UNKNOWN;
    // }
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
    solution.x.x = B_null_space * solution.x.x + x0;
  }

  return solution;
}

}  // namespace quadratic_programs
}  // namespace conex
