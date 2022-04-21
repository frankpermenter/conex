#include <Eigen/Dense>

#include "conex/kkt_solver_interface.h"
#include "conex/kkt_subsystem.h"

namespace conex {

class TreeSolver : public KKTSolverBase {
 public:
  void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                      bool in_original_order) const {

    if (in_original_order) {
      Eigen::PermutationMatrix<-1> P(number_of_variables());
      P.indices() = Eigen::Map<const Eigen::VectorXi>(variable_to_elimination_position_.data(),
                                         number_of_variables());
      b = P * b;
    }

    for (auto root : roots_) {
      root->ApplyInverseOfLeftFactor(b);
      root->ApplyInverseOfRightFactor(b);
    }

    if (in_original_order) {
      Eigen::PermutationMatrix<-1> P(number_of_variables());
      P.indices() = Eigen::Map<const Eigen::VectorXi>(variable_to_elimination_position_.data(),
                                         number_of_variables());
      b = P.transpose() * b;
    }
  }

  void DoAssemble() override { 
    for (auto root : roots_) {
      root->Assemble(); 
    }
  }

  bool DoFactor() override {
    for (auto root : roots_) {
      root->AssembleAndFactor();
    }
    return true;
  }

  void MakeTree(std::vector<int> parent) {
    for (size_t i = 0; i < parent.size(); ++i) {
      if (parent[i] > 0) {
        subsystems_.at(parent[i])->AddChild(subsystems_.at(i));
      } else {
        roots_.push_back(subsystems_.at(i));
      }
    }
    // Post-order
    variable_to_elimination_position_.resize(number_of_variables());
    int first = 0;
    for (auto r : roots_) {
      r->ComputePostOrdering(first, &variable_to_elimination_position_);
    }
    for (auto s : subsystems_) {
      s->SetVariableOrdering(variable_to_elimination_position_);
    }
  }

  int number_of_variables() const {
    int max = 0;
    for (auto s : subsystems_) {
      const auto& sn =  s->supernodes();
      double max_s = *std::max_element(sn.begin(), sn.end());
      if (max_s > max) {
        max = max_s;
      }
    }
    return max + 1;
  }

  Eigen::MatrixXd DoKKTMatrix(bool permute_to_elimination_order = true) const {
    int num_vars = number_of_variables(); 
    Eigen::MatrixXd M(num_vars, num_vars);
    M.setZero();
    for (auto root : roots_) {
      root->MakeKKTMatrix(&M);
      M = M.selfadjointView<Eigen::Lower>();
    }
    if (permute_to_elimination_order) {
      return M;
    }
    Eigen::PermutationMatrix<-1> P(number_of_variables());
    P.indices() = Eigen::Map<const Eigen::VectorXi>(variable_to_elimination_position_.data(),
                                       number_of_variables());
    return P.transpose() * M * P;
  }

  void AddSubsystem(KKTSubsystem* system) { 
    subsystems_.push_back(system);
  }
 private:
  std::vector<KKTSubsystem*> roots_;
  std::vector<KKTSubsystem*> subsystems_;
  std::vector<int> variable_to_elimination_position_;
};

} // namespace conex
