#include "two_dimensional_lp.h"

#include <iostream>
#include <vector>

#define DUMP(x)                                              \
  std::cerr << __FILE__ << " line " << __LINE__ << std::endl \
            << #x ":" << std::endl                           \
            << x << std::endl;
namespace {
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class Matrix {
  Matrix(int num_rows, int num_cols)
      : m_(num_rows * num_cols), num_rows_(num_rows), num_cols_(num_cols) {}

  void SetRandom() {
    for (auto& a : m_) {
      a = (double)rand() / RAND_MAX;
    }
  }

 private:
  std::vector<double> m_;
  int num_rows_;
  int num_cols_;
};

struct Data {
  MatrixXd matrix;
  VectorXd affine;
};

class Vertex {
 public:
  Eigen::Vector2d x;
  int facet_inequalities[2];
  Vertex(const Data& data, int inequality_1, int inequality_2) {
    MatrixXd B(2, 2);
    VectorXd f(2);
    B.row(0) = data.matrix.row(inequality_1);
    f.row(0) = data.affine.row(inequality_1);
    B.row(1) = data.matrix.row(inequality_2);
    f.row(1) = data.affine.row(inequality_2);
    x = B.inverse() * f;
    facet_inequalities[0] = inequality_1;
    facet_inequalities[1] = inequality_2;
  }
};

void PartitionUsingHyperplane(const Data& data, int i,
                              const vector<Vertex>& vertices,
                              vector<Vertex>* satisfied,
                              vector<Vertex>* violated) {
  int cnt = 0;
  violated->clear();
  satisfied->clear();
  for (const auto& v : vertices) {
    double slack = data.matrix.row(i) * v.x - data.affine(i);
    if (slack > 0) {
      violated->push_back(v);
    } else {
      satisfied->push_back(v);
    }
  }
}

bool IsFeasible(const Data& data, const Vertex& v, int start, int number) {
  for (int i = start; i < start + number; i++) {
    double slack = data.matrix.row(i) * v.x - data.affine(i);
    if (slack > 1e-12) {
      return false;
    }
  }
  return true;
}

void FindVertices(const Data data, int ineq_start, vector<Vertex>& vertices) {
  for (int ineq = ineq_start; ineq < data.matrix.rows(); ineq++) {
    // For each violated vertex, find intersection
    // of facet inequalities with new hyperplane

    vector<Vertex> keep;
    vector<Vertex> remove;
    PartitionUsingHyperplane(data, ineq, vertices, &keep, &remove);
    for (auto v : remove) {
      Vertex v1(data, v.facet_inequalities[0], ineq);
      Vertex v2(data, v.facet_inequalities[1], ineq);

      if (!std::isnan(v1.x.norm()) && IsFeasible(data, v1, 0, ineq)) {
        keep.push_back(v1);
      }
      if (!std::isnan(v2.x.norm()) && IsFeasible(data, v2, 0, ineq)) {
        keep.push_back(v2);
      }
    }
    vertices = keep;
  }
}

// Solves Ax <= b,  lb < x, x(0) + x(1) < ub;
auto DoMain(MatrixXd A, VectorXd b, double ub_x0, double ub_x1, double lb_x0,
            double lb_x1) {
  Data data;
  data.matrix.resize(A.rows() + 4, 2);
  data.affine.resize(A.rows() + 4, 1);

  data.matrix.topRows(4) << 0, -1, -1, 0, 1, 0, 0, 1;

  data.affine.topRows(4) << -lb_x1, -lb_x0, ub_x0, ub_x1;

  data.matrix.bottomRows(A.rows()) = A;
  data.affine.bottomRows(A.rows()) = b;

  vector<Vertex> vertices;
  vertices.emplace_back(data, 0, 1);
  vertices.emplace_back(data, 0, 2);
  vertices.emplace_back(data, 2, 3);
  vertices.emplace_back(data, 1, 3);
  FindVertices(data, 4, vertices);
  return vertices;
}

void GetInfinityNorm(const Eigen::MatrixXd& d0, const Eigen::MatrixXd& d1,
                     const Eigen::MatrixXd& d2, double bound,
                     Eigen::MatrixXd* A, Eigen::VectorXd* b) {
  // d0 + d1x + d2 y <= bound
  //
  int m = d0.rows();
  A->resize(2 * m, 2);
  b->resize(2 * m, 1);
  b->head(m).setConstant(bound);
  b->head(m) -= d0;
  A->topRows(m) << d1, d2;

  // d0 + d1x + d2 y >= -bound
  // =>
  // -d0 - d1x - d2y <= bound
  A->bottomRows(m) << -d1, -d2;
  b->tail(m).setConstant(bound);
  b->tail(m) += d0;
}

int MaxLogInvSqrtMu(const std::vector<Vertex>& vertices) {
  double max_log_x1 = -1;
  int arg_max = -1;

  int i = 0;
  for (const auto& v : vertices) {
    if (std::fabs(std::log(v.x(0))) > max_log_x1) {
      arg_max = i;
      max_log_x1 = std::fabs(std::log(v.x(0)));
    }
    i++;
  }
  return arg_max;
}

// This doesn't make sense: x/y is not
// convex and is hence not maximized at a vertex.
int MinTheta(const std::vector<Vertex>& vertices) {
  int i = 0;
  int arg_min = -1;
  double min_val = 1e16;

  for (const auto& v : vertices) {
    if (v.x(1) < min_val) {
      arg_min = i;
      min_val = v.x(1) / v.x(0);
    }
    i++;
  }
  return arg_min;
}

// max.  (inv_sqrt_mu)^2 / (inv_sqrt_mu * theta)
//        = inv_sqrt_mu / theta
//
//        = min. theta * sqrt(mu)
int MinRatio(const std::vector<Vertex>& vertices) {
  int i = 0;
  int arg_min = -1;
  double min_val = 1e15;

  for (const auto& v : vertices) {
    double theta = v.x(1) / v.x(0);
    double val = theta / v.x(0) + 1.0 / v.x(0);
    if (val < min_val) {
      arg_min = i;
      min_val = val;
    }
    i++;
  }
  return arg_min;
}

int MinLinearFunction(const std::vector<Vertex>& vertices,
                      const Eigen::Vector2d& w) {
  int i = 0;
  int arg_min = 0;
  double min_val = vertices.at(0).x.dot(w);

  for (const auto& v : vertices) {
    double val = v.x.dot(w);
    if (val < min_val) {
      arg_min = i;
      min_val = val;
    }
    i++;
  }
  return arg_min;
}

}  // namespace

namespace conex {
namespace quadratic_programs {

Eigen::MatrixXd InfinityNorm(const Eigen::MatrixXd& d0,
                             const Eigen::MatrixXd& d1,
                             const Eigen::MatrixXd& d2, double bound,
                             const Limits& limit) {
  MatrixXd A;
  VectorXd b;
  GetInfinityNorm(d0, d1, d2, bound, &A, &b);
  auto vertices = DoMain(
      A, b, limit.inv_sqrt_mu_ub /*ub of x0*/, limit.theta_times_inv_sqrt_mu_ub,
      limit.inv_sqrt_mu_lb /*ub of x0*/, limit.theta_times_inv_sqrt_mu_lb);
  if (vertices.size() == 0) {
    DUMP("FAILED");
    DUMP(A);
    DUMP(b);
    DUMP(limit.inv_sqrt_mu_ub);
    DUMP(limit.inv_sqrt_mu_lb);
    DUMP(limit.theta_times_inv_sqrt_mu_lb);
    DUMP(limit.theta_times_inv_sqrt_mu_ub);
    return VectorXd();
  }

  // int opt = MaxLogInvSqrtMu(vertices);
  // int opt = MinRatio(vertices);

  Eigen::Vector2d w;
  w << limit.inv_sqrt_mu_weight, limit.theta_weight;
  int opt = MinLinearFunction(vertices, w);
  return vertices.at(opt).x;
}

}  // namespace quadratic_programs
}  // namespace conex
