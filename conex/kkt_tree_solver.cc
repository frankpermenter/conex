#include "conex/kkt_tree_solver.h"

namespace conex {
using T = SymmetricLinearSystemTreeSolver;

void T::DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                       bool in_original_order) const {
  if (in_original_order) {
    Eigen::PermutationMatrix<-1> P(number_of_variables());
    P.indices() = Eigen::Map<const Eigen::VectorXi>(
        variable_to_elimination_position_.data(), number_of_variables());
    b = P * b;
  }

  for (auto root : roots_) {
    root->ApplyInverseOfLeftFactor(b);
    root->ApplyInverseOfRightFactor(b);
  }

  if (in_original_order) {
    Eigen::PermutationMatrix<-1> P(number_of_variables());
    P.indices() = Eigen::Map<const Eigen::VectorXi>(
        variable_to_elimination_position_.data(), number_of_variables());
    b = P.transpose() * b;
  }
}

void T::DoAssemble() {
  for (auto root : roots_) {
    root->Assemble();
  }
}

bool T::DoFactor() {
  for (auto root : roots_) {
    root->AssembleAndFactor();
  }
  return true;
}

void T::MakeTreeHelper(const std::vector<int>& parent) {
  CONEX_DEMAND(parent.size() == subsystems_.size(), "Size of parent vector must equal number of subsystems.");
  roots_.clear();
  for (auto s : subsystems_) {
    s->Reset();
  }
  for (size_t i = 0; i < parent.size(); ++i) {
    if (parent[i] >= 0) {
      CONEX_DEMAND(parent[i] != static_cast<int>(i), "Tree is malformed: node cannot be own parent.");
      subsystems_.at(parent[i])->AddChild(subsystems_.at(i));
    } else {
      roots_.push_back(subsystems_.at(i));
    }
  }

  // Set supernodes from parent.
  for (size_t i = 0; i < parent.size(); ++i) {
    if (parent[i] >= 0) {
      std::vector<int> v1 = subsystems_.at(parent[i])->shared_variables();
      std::vector<int> v2 = subsystems_.at(i)->shared_variables();
      std::sort(v1.begin(), v1.end());
      std::sort(v2.begin(), v2.end());
      std::vector<int> separators;
      std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(),
                            std::back_inserter(separators));
      subsystems_.at(i)->SetSeparators(separators);

      std::vector<int> supernodes;
      std::set_difference(v2.begin(), v2.end(), separators.begin(),
                          separators.end(), std::back_inserter(supernodes));
      subsystems_.at(i)->SetSupernodes(supernodes);
    } else {
      std::vector<int> v2 = subsystems_.at(i)->shared_variables();
      std::sort(v2.begin(), v2.end());
      subsystems_.at(i)->SetSupernodes(v2);
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

bool T::CheckForZeroPivot(const std::vector<int>& parent, 
                          std::vector<int>* index_of_zero_pivot) {
  index_of_zero_pivot->clear();
  MakeTreeHelper(parent);
  Assemble();
  int i = 0; 
  for (auto r : subsystems_) {
    if (r->supernodes().size() > 0) {
      Eigen::MatrixXd T = r->supernode_submatrix();
      T = T.cwiseProduct(T);
      bool zero_pivot = T.colwise().sum().minCoeff() == 0;
      if (zero_pivot) {
        index_of_zero_pivot->push_back(i);
      }
    }
    i++;
  }
  return index_of_zero_pivot->size() > 0;
}

void T::MakeTree(const std::vector<int>& parent) {
  MakeTreeHelper(parent);
  std::vector<int> index_of_zero_pivot;
  if (CheckForZeroPivot(parent, &index_of_zero_pivot)) {
    throw std::runtime_error("Invalid tree: zero pivot detected.");
  }
}

void T::RepairTreeInPlace(std::vector<int>* parent_ptr) {
  auto& parent = *parent_ptr;
  bool regenerate = true;
  std::vector<int> zero_pivot_indices;
  while (regenerate) {
    regenerate = false;
    bool zero_pivot = CheckForZeroPivot(parent, &zero_pivot_indices);
    if (zero_pivot) {
      for (auto i : zero_pivot_indices) {
        int parent_index = parent[i];
        int parent_of_parent = parent[parent_index];
        parent[i] = parent_of_parent;
        parent[parent_index] = i;
        regenerate = true;
      }
    }
  }
}

int T::number_of_variables() const {
  int max = 0;
  for (auto s : subsystems_) {
    const auto& sn = s->supernodes();
    if (sn.size() > 0) {
      double max_s = *std::max_element(sn.begin(), sn.end());
      if (max_s > max) {
        max = max_s;
      }
    }
  }
  return max + 1;
}

Eigen::MatrixXd T::DoKKTMatrix(bool permute_to_elimination_order) const {
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
  P.indices() = Eigen::Map<const Eigen::VectorXi>(
      variable_to_elimination_position_.data(), number_of_variables());
  return P.transpose() * M * P;
}

void T::AddSubsystem(KKTSubsystem* system) { subsystems_.push_back(system); }

}  // namespace conex
