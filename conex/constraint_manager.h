#pragma once

#include <any>
#include <list>
#include "conex/equality_constraint.h"
#include "conex/kkt_system_assembler.h"
#include "conex/newton_step.h"

namespace conex {

inline int IsUnique(int N, const std::vector<int>& x) {
  Eigen::VectorXd y(N);
  y.setZero();
  for (auto& xi : x) {
    if (xi >= N) {
      return false;
    }
    y(xi)++;
    if (y(xi) > 1) {
      return false;
    }
  }
  return true;
}

template <typename Container>
class ConstraintManager {
 public:
  ConstraintManager(int max_number_of_variables)
      : max_number_of_variables_(max_number_of_variables),
        dual_variable_start_(max_number_of_variables_) {}

  ConstraintManager(){};

  void SetNumberOfVariables(int N) {
    max_number_of_variables_ = N;
    dual_variable_start_ = N;
  }
  
  int GetNumberOfVariables() {
    return max_number_of_variables_;
  }

  int SizeOfKKTSystem() {
    int num_dual_vars = 0;
    for (auto dv : dual_vars) {
      num_dual_vars += dv.size();
    }
    return max_number_of_variables_ + num_dual_vars;
  };

  template <typename T>
  bool AddConstraint(T&& x) {
    std::vector<int> clique(max_number_of_variables_);
    for (size_t i = 0; i < clique.size(); i++) {
      clique[i] = i;
    }
    AddConstraint(x, clique);
    return true;
  }

  template <typename T>
  bool AddConstraint(T&& x, const std::vector<int>& variables) {
    if (!IsUnique(max_number_of_variables_, variables)) {
      return false;
    }
    eqs.emplace_back(x);
    cliques.push_back(variables);
    dual_vars.push_back({});
    return true;
  }

  bool AddEqualityConstraint(EqualityConstraints&& x,
                             const std::vector<int>& variables) {
    if (!IsUnique(max_number_of_variables_, variables)) {
      return false;
    }
    eqs.emplace_back(x);
    cliques.push_back(variables);
    const int m = x.SizeOfDualVariable();
    dual_vars.push_back({});
    for (int i = 0; i < m; i++) {
      cliques.back().push_back(i + dual_variable_start_);
      dual_vars.back().push_back(i + dual_variable_start_);
    }
    dual_variable_start_ += m;
    return true;
  }

  bool AddEqualityConstraint(EqualityConstraints&& x) {
    std::vector<int> clique(max_number_of_variables_);
    for (size_t i = 0; i < clique.size(); i++) {
      clique[i] = i;
    }
    AddEqualityConstraint(std::forward<EqualityConstraints>(x), clique);
    return true;
  }

  // Use a list so that we do not trigger reallocations.
  std::list<Container> eqs;
  std::vector<std::vector<int>> cliques;
  std::vector<std::vector<int>> dual_vars;

 private:
  int max_number_of_variables_ = 0;
  int dual_variable_start_ = 0;
};


inline Eigen::VectorXd ExtractVars(const Eigen::VectorXd& x, std::vector<int> indices) {
  Eigen::VectorXd z(indices.size());
  int cnt = 0;
  for (auto i : indices) {
    z(cnt++) = x(i);
  }
  return z;
}

template<typename Container>
void PrepareStep(ConstraintManager<Container>* kkt,
                 const StepOptions& newton_step_parameters, const Ref& y,
                 StepInfo* info) {
  StepInfo info_i;
  info_i.normsqrd = 0;
  info_i.norminfd = 0;
  info->normsqrd = 0;
  info->norminfd = -1;
  int i = 0;
  for (auto& ci : kkt->eqs) {
    // TODO(FrankPermenter): Remove creation of these maps.
    auto ysegment = ExtractVars(y, kkt->cliques.at(i));
    Eigen::Map<Eigen::MatrixXd, Eigen::Aligned> z(ysegment.data(),
                                                  ysegment.size(), 1);
    PrepareStep(&ci.constraint, newton_step_parameters, z, &info_i);
    if (info_i.norminfd > info->norminfd) {
      info->norminfd = info_i.norminfd;
    }
    info->normsqrd += info_i.normsqrd;
    i++;
  }
}

template<typename Container>
void TakeStep(ConstraintManager<Container>* kkt,
              const StepOptions& newton_step_parameters) {
  for (auto& ci : kkt->eqs) {
    TakeStep(&ci.constraint, newton_step_parameters);
  }
}

template<typename Container>
void AssembleSchurComplement(ConstraintManager<Container>* kkt,
                             SelfDualEmbeddingSystem* s) {
  s->setZero();
  int i = 0;
  for (auto& ci : kkt->eqs) {
    auto* rhs_i = &ci.kkt_assembler.schur_complement_data;
    s->inner_product_of_c_and_w += rhs_i->inner_product_of_c_and_w;
    s->inner_product_of_c_and_e += rhs_i->inner_product_of_c_and_e;
    s->inner_product_of_c_and_Qc += rhs_i->inner_product_of_c_and_Qc;
    s->inner_product_of_c_and_Qe += rhs_i->inner_product_of_c_and_Qe;
    int cnt = 0;
    for (auto k : kkt->cliques.at(i)) {
      s->AW(k) += rhs_i->AW(cnt);
      s->AQc(k) += rhs_i->AQc(cnt);
      s->AQe(k) += rhs_i->AQe(cnt);
      s->Ae(k) += rhs_i->Ae(cnt);
      cnt++;
    }
    i++;
  }
}



}  // namespace conex
