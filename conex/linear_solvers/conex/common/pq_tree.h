#pragma once
#include <vector>

namespace conex {

class PQTree {
 public:
  // Build a universal tree: a single P-node with n leaves labeled 0..n-1.
  explicit PQTree(int n);
  ~PQTree();

  // Reduce the tree so that the elements of `subset` are contiguous in every
  // frontier permutation.  Returns false if the constraint is unsatisfiable
  // (the tree is left unchanged in that case).
  bool AddConstraint(const std::vector<int>& subset);

  // Try each constraint in order; skip (and count) those that fail.
  // Returns the number of constraints successfully applied.
  int AddConstraintsBestEffort(const std::vector<std::vector<int>>& subsets);

  // Return the current left-to-right leaf order (a permutation of 0..n-1).
  std::vector<int> GetPermutation() const;

 private:
  struct Node;

  enum NodeType { LEAF, P_NODE, Q_NODE };
  enum Label { EMPTY, FULL, PARTIAL };

  struct Node {
    NodeType type;
    int leaf_id;  // Only meaningful for LEAF nodes.
    std::vector<Node*> children;
    Node* parent;
    Label label;
    int pertinent_child_count;
    int pertinent_leaf_count;

    Node() : type(P_NODE), leaf_id(-1), parent(nullptr),
             label(EMPTY), pertinent_child_count(0),
             pertinent_leaf_count(0) {}
  };

  // Deep-copy the entire tree rooted at `src`.
  Node* DeepCopy(const Node* src);
  void FreeTree(Node* root);

  // Booth-Lueker phases.
  bool Bubble(const std::vector<int>& subset);
  bool Reduce(const std::vector<int>& subset);

  // Template matching helpers.
  bool ReduceP(Node* node, bool is_pertinent_root);
  bool ReduceQ(Node* node, bool is_pertinent_root);

  // Collect leaves in left-to-right order.
  void CollectLeaves(const Node* node, std::vector<int>* out) const;

  // Update parent pointers of `node`'s children.
  void LinkChildren(Node* node);

  // Count full children of a node.
  int CountFull(const Node* node) const;
  int CountPartial(const Node* node) const;

  // Find the single/double partial child(ren).
  Node* FindPartialChild(Node* node) const;

  // Ensure a partial Q-node has FULL children at the front.
  void GatherQChildren(Node* partial_q);

  Node* root_;
  std::vector<Node*> leaves_;  // leaves_[i] is the leaf for element i.
  std::vector<Node*> all_nodes_;  // For memory management.

  Node* AllocNode();
};

}  // namespace conex
