#include "conex/pq_tree.h"

#include <algorithm>
#include <cassert>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace conex {

PQTree::Node* PQTree::AllocNode() {
  auto* n = new Node();
  all_nodes_.push_back(n);
  return n;
}

PQTree::~PQTree() {
  for (Node* n : all_nodes_) delete n;
}

PQTree::PQTree(int n) : root_(nullptr) {
  if (n == 0) return;
  leaves_.resize(n);
  root_ = AllocNode();
  root_->type = P_NODE;
  root_->children.resize(n);
  for (int i = 0; i < n; ++i) {
    Node* leaf = AllocNode();
    leaf->type = LEAF;
    leaf->leaf_id = i;
    leaf->parent = root_;
    root_->children[i] = leaf;
    leaves_[i] = leaf;
  }
}

PQTree::Node* PQTree::DeepCopy(const Node* src) {
  if (!src) return nullptr;
  Node* dst = new Node();
  dst->type = src->type;
  dst->leaf_id = src->leaf_id;
  dst->parent = nullptr;
  dst->label = src->label;
  dst->pertinent_child_count = 0;
  dst->pertinent_leaf_count = 0;
  dst->children.resize(src->children.size());
  for (size_t i = 0; i < src->children.size(); ++i) {
    dst->children[i] = DeepCopy(src->children[i]);
    dst->children[i]->parent = dst;
  }
  return dst;
}

void PQTree::FreeTree(Node* root) {
  if (!root) return;
  for (Node* c : root->children) FreeTree(c);
  delete root;
}

void PQTree::CollectLeaves(const Node* node, std::vector<int>* out) const {
  if (!node) return;
  if (node->type == LEAF) {
    out->push_back(node->leaf_id);
    return;
  }
  for (const Node* c : node->children) {
    CollectLeaves(c, out);
  }
}

void PQTree::LinkChildren(Node* node) {
  for (Node* c : node->children) c->parent = node;
}

int PQTree::CountFull(const Node* node) const {
  int cnt = 0;
  for (const Node* c : node->children) {
    if (c->label == FULL) ++cnt;
  }
  return cnt;
}

int PQTree::CountPartial(const Node* node) const {
  int cnt = 0;
  for (const Node* c : node->children) {
    if (c->label == PARTIAL) ++cnt;
  }
  return cnt;
}

PQTree::Node* PQTree::FindPartialChild(Node* node) const {
  for (Node* c : node->children) {
    if (c->label == PARTIAL) return c;
  }
  return nullptr;
}

std::vector<int> PQTree::GetPermutation() const {
  std::vector<int> perm;
  perm.reserve(leaves_.size());
  CollectLeaves(root_, &perm);
  return perm;
}

// ---------------------------------------------------------------------------
// Bubble: reset all labels, mark pertinent leaves FULL.
// ---------------------------------------------------------------------------
bool PQTree::Bubble(const std::vector<int>& subset) {
  std::queue<Node*> q;
  if (root_) q.push(root_);
  while (!q.empty()) {
    Node* n = q.front(); q.pop();
    n->label = EMPTY;
    n->pertinent_child_count = 0;
    n->pertinent_leaf_count = 0;
    for (Node* c : n->children) q.push(c);
  }
  for (int idx : subset) {
    leaves_[idx]->label = FULL;
    leaves_[idx]->pertinent_leaf_count = 1;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Reduce: bottom-up label + template match.
//
// Strategy: collect pertinent leaves into a queue.  For each leaf, propagate
// pertinent_leaf_count to parent; when a parent has accumulated counts from
// all pertinent children, it's ready to process.  We detect "all pertinent
// children reported" by counting: a child is pertinent if its
// pertinent_leaf_count > 0 after propagation or if it was already labeled
// non-EMPTY.
// ---------------------------------------------------------------------------
bool PQTree::Reduce(const std::vector<int>& subset) {
  if (subset.empty()) return true;
  if (static_cast<int>(subset.size()) == static_cast<int>(leaves_.size()))
    return true;

  // Bottom-up BFS from pertinent leaves.
  std::queue<Node*> ready;
  std::unordered_set<Node*> enqueued;
  for (int idx : subset) {
    ready.push(leaves_[idx]);
    enqueued.insert(leaves_[idx]);
  }

  while (!ready.empty()) {
    Node* node = ready.front(); ready.pop();

    if (node->type == LEAF) {
      // Propagate to parent.
      Node* p = node->parent;
      if (!p) continue;
      p->pertinent_child_count++;
      p->pertinent_leaf_count += node->pertinent_leaf_count;
      // Count how many of p's children are pertinent.
      int expected = 0;
      for (Node* c : p->children) {
        if (c->pertinent_leaf_count > 0 || c->label != EMPTY) ++expected;
      }
      if (p->pertinent_child_count >= expected && !enqueued.count(p)) {
        enqueued.insert(p);
        ready.push(p);
      }
      continue;
    }

    // Interior node — determine label.
    int full_cnt = CountFull(node);
    int partial_cnt = CountPartial(node);
    int total = static_cast<int>(node->children.size());
    int empty_cnt = total - full_cnt - partial_cnt;

    bool is_pertinent_root =
        (node->parent == nullptr) ||
        (node->pertinent_leaf_count == static_cast<int>(subset.size()));

    // Label this node.
    if (full_cnt == total) {
      node->label = FULL;
    } else if (empty_cnt == total) {
      node->label = EMPTY;
    } else {
      // Has a mix → PARTIAL; apply templates.
      node->label = PARTIAL;
      bool ok = (node->type == P_NODE)
                    ? ReduceP(node, is_pertinent_root)
                    : ReduceQ(node, is_pertinent_root);
      if (!ok) return false;
    }

    // Propagate to parent.
    Node* p = node->parent;
    if (!p) continue;
    p->pertinent_child_count++;
    p->pertinent_leaf_count += node->pertinent_leaf_count;
    int expected = 0;
    for (Node* c : p->children) {
      if (c->pertinent_leaf_count > 0 || c->label != EMPTY) ++expected;
    }
    if (p->pertinent_child_count >= expected && !enqueued.count(p)) {
      enqueued.insert(p);
      ready.push(p);
    }
  }

  return true;
}

// ---------------------------------------------------------------------------
// Helpers: split a Q-node's children into (full-end, partial, interior
// empties, partial, full-end), absorbing partial Q-children inline.
// Returns the FULL-contiguous children vector, or empty on failure.
//
// For a valid Q-node, all FULL+PARTIAL children must be clustered at one
// or both ends.  We find the arrangement by scanning from both sides.
// ---------------------------------------------------------------------------

// Gather the FULL children of a partial Q-node into a contiguous run at
// the front of its children list (the convention).  Returns the children
// vector with FULL nodes at front, EMPTY at back, absorbing sub-Q-nodes.
void PQTree::GatherQChildren(Node* partial_q) {
  auto& ch = partial_q->children;
  if (!ch.empty() && ch.front()->label == FULL) return;
  if (!ch.empty() && ch.back()->label == FULL) {
    std::reverse(ch.begin(), ch.end());
  }
}

// ---------------------------------------------------------------------------
// P-node templates.
//
// Convention: after template application, a PARTIAL node's children are
// ordered with FULL children/content at the FRONT.
// ---------------------------------------------------------------------------
bool PQTree::ReduceP(Node* node, bool is_pertinent_root) {
  // Categorise children.
  std::vector<Node*> full_ch, partial_ch, empty_ch;
  for (Node* c : node->children) {
    switch (c->label) {
      case FULL:    full_ch.push_back(c); break;
      case PARTIAL: partial_ch.push_back(c); break;
      default:      empty_ch.push_back(c); break;
    }
  }
  int nf = static_cast<int>(full_ch.size());
  int np = static_cast<int>(partial_ch.size());
  int ne = static_cast<int>(empty_ch.size());

  if (np > 2) return false;
  if (np == 2 && !is_pertinent_root) return false;
  if (np == 1 && ne == 0 && nf == 0) {
    // Single partial child only — shouldn't happen (node would have been
    // the partial child itself).  Just propagate.
    node->label = PARTIAL;
    return true;
  }

  // ------------------------------------------------------------------
  // Case: no partial children (P2 template).
  // ------------------------------------------------------------------
  if (np == 0) {
    if (ne == 0) {
      node->label = FULL;
      return true;
    }
    if (is_pertinent_root) {
      // Pertinent root: group full children together at front.
      node->children.clear();
      if (nf > 1) {
        Node* fg = AllocNode();
        fg->type = P_NODE; fg->label = FULL; fg->parent = node;
        fg->children = full_ch; LinkChildren(fg);
        node->children.push_back(fg);
      } else {
        for (Node* c : full_ch) node->children.push_back(c);
      }
      for (Node* c : empty_ch) node->children.push_back(c);
      return true;
    }
    // Not pertinent root: replace this P-node with a Q-node [full | empty].
    Node* qn = AllocNode();
    qn->type = Q_NODE; qn->label = PARTIAL; qn->parent = node->parent;
    qn->pertinent_leaf_count = node->pertinent_leaf_count;
    // Replace in parent.
    if (node->parent) {
      for (auto& s : node->parent->children) {
        if (s == node) { s = qn; break; }
      }
    }
    if (root_ == node) root_ = qn;
    // Full group.
    if (nf > 1) {
      Node* fg = AllocNode();
      fg->type = P_NODE; fg->label = FULL; fg->parent = qn;
      fg->children = full_ch; LinkChildren(fg);
      qn->children.push_back(fg);
    } else if (nf == 1) {
      full_ch[0]->parent = qn; qn->children.push_back(full_ch[0]);
    }
    // Empty group.
    if (ne > 1) {
      Node* eg = AllocNode();
      eg->type = P_NODE; eg->label = EMPTY; eg->parent = qn;
      eg->children = empty_ch; LinkChildren(eg);
      qn->children.push_back(eg);
    } else if (ne == 1) {
      empty_ch[0]->parent = qn; qn->children.push_back(empty_ch[0]);
    }
    // Keep node->parent intact for bottom-up propagation.
    return true;
  }

  // ------------------------------------------------------------------
  // Case: 1 partial child (P3/P5/P6 templates).
  // ------------------------------------------------------------------
  if (np == 1) {
    Node* pc = partial_ch[0];
    // Ensure partial child (if Q-node) has FULL at front.
    if (pc->type == Q_NODE) GatherQChildren(pc);

    if (is_pertinent_root) {
      // Merge full children into partial child's full end.
      // Result: node's children = [partial_child_expanded, empty_children].
      // The partial child absorbs the full children at its front.
      if (pc->type == Q_NODE) {
        // Prepend full children to partial child's children.
        std::vector<Node*> new_pc;
        for (Node* c : full_ch) { c->parent = pc; new_pc.push_back(c); }
        for (Node* c : pc->children) new_pc.push_back(c);
        pc->children = std::move(new_pc);
      } else {
        // Wrap in a new Q-node: [full..., pc].
        Node* qn = AllocNode();
        qn->type = Q_NODE; qn->label = PARTIAL; qn->parent = node;
        for (Node* c : full_ch) { c->parent = qn; qn->children.push_back(c); }
        pc->parent = qn; qn->children.push_back(pc);
        pc = qn;
      }
      node->children.clear();
      pc->parent = node;
      node->children.push_back(pc);
      for (Node* c : empty_ch) node->children.push_back(c);
      return true;
    }

    // Not pertinent root: build Q-node [full-group, pc's innards..., empty-group].
    Node* qn = AllocNode();
    qn->type = Q_NODE; qn->label = PARTIAL; qn->parent = node->parent;
    qn->pertinent_leaf_count = node->pertinent_leaf_count;
    if (node->parent) {
      for (auto& s : node->parent->children) {
        if (s == node) { s = qn; break; }
      }
    }
    if (root_ == node) root_ = qn;

    // Full group at front.
    if (nf > 1) {
      Node* fg = AllocNode();
      fg->type = P_NODE; fg->label = FULL; fg->parent = qn;
      fg->children = full_ch; LinkChildren(fg);
      qn->children.push_back(fg);
    } else if (nf == 1) {
      full_ch[0]->parent = qn; qn->children.push_back(full_ch[0]);
    }
    // Inline partial child.
    if (pc->type == Q_NODE) {
      for (Node* c : pc->children) { c->parent = qn; qn->children.push_back(c); }
    } else {
      pc->parent = qn; qn->children.push_back(pc);
    }
    // Empty group at back.
    if (ne > 1) {
      Node* eg = AllocNode();
      eg->type = P_NODE; eg->label = EMPTY; eg->parent = qn;
      eg->children = empty_ch; LinkChildren(eg);
      qn->children.push_back(eg);
    } else if (ne == 1) {
      empty_ch[0]->parent = qn; qn->children.push_back(empty_ch[0]);
    }
    // Keep node->parent intact for bottom-up propagation.
    return true;
  }

  // ------------------------------------------------------------------
  // Case: 2 partial children (P4 template, pertinent root only).
  // ------------------------------------------------------------------
  if (np == 2) {
    Node* p1 = partial_ch[0];
    Node* p2 = partial_ch[1];
    // Merged order: [...p1..., full_group, ...p2...]
    // p1 needs FULL at back (adjacent to full_group).
    // p2 needs FULL at front (adjacent to full_group).
    if (p1->type == Q_NODE) {
      GatherQChildren(p1);  // FULL at front
      std::reverse(p1->children.begin(), p1->children.end());  // FULL at back
    }
    if (p2->type == Q_NODE) {
      GatherQChildren(p2);  // FULL at front — correct
    }

    // Build Q-node: [...p1..., full-group, ...p2...].
    Node* qn = AllocNode();
    qn->type = Q_NODE; qn->label = PARTIAL; qn->parent = node;

    if (p1->type == Q_NODE) {
      for (Node* c : p1->children) { c->parent = qn; qn->children.push_back(c); }
    } else {
      p1->parent = qn; qn->children.push_back(p1);
    }
    for (Node* c : full_ch) { c->parent = qn; qn->children.push_back(c); }
    if (p2->type == Q_NODE) {
      for (Node* c : p2->children) { c->parent = qn; qn->children.push_back(c); }
    } else {
      p2->parent = qn; qn->children.push_back(p2);
    }

    node->children.clear();
    node->children.push_back(qn);
    for (Node* c : empty_ch) node->children.push_back(c);
    return true;
  }

  return false;
}

// ---------------------------------------------------------------------------
// Q-node templates.
//
// Q-node children are in a fixed linear order (may be reversed).
// FULL/PARTIAL children must form a contiguous group at one or both ends.
// After reduction, FULL content is at the FRONT.
// ---------------------------------------------------------------------------
bool PQTree::ReduceQ(Node* node, bool is_pertinent_root) {
  auto& ch = node->children;
  int n = static_cast<int>(ch.size());
  if (n == 0) return true;

  int full_cnt = CountFull(node);
  int partial_cnt = CountPartial(node);

  if (partial_cnt > 2) return false;
  if (!is_pertinent_root && partial_cnt > 1) return false;

  // Q1: all full.
  if (full_cnt == n) { node->label = FULL; return true; }

  // Validate arrangement: non-EMPTY children must be contiguous at the LEFT
  // end.  Pattern: [FULL*, PARTIAL?, EMPTY*] — possibly with a second PARTIAL
  // at the boundary if this is the pertinent root (but that case has the
  // pattern [FULL*, PARTIAL, EMPTY*, PARTIAL, EMPTY*] which still has all
  // non-empty at the left).
  //
  // The key invariant: no EMPTY child may appear between two non-EMPTY children.
  // Equivalently: [non-EMPTY..., EMPTY...] with at most the allowed number of
  // PARTIAL children in the non-EMPTY prefix.
  auto valid_arrangement = [&]() -> bool {
    // Non-EMPTY children must all be at the left end.
    // Scan: [FULL*, PARTIAL?, FULL*, PARTIAL?, EMPTY*]
    // (FULL and PARTIAL are interleaved at the left, all EMPTY at right)
    int i = 0;
    // Left FULL run.
    while (i < n && ch[i]->label == FULL) ++i;
    // Optional PARTIAL.
    if (i < n && ch[i]->label == PARTIAL) ++i;
    // More FULL (between two partials or after one).
    while (i < n && ch[i]->label == FULL) ++i;
    // Optional second PARTIAL (pertinent root only).
    if (i < n && ch[i]->label == PARTIAL) ++i;
    // Remaining must all be EMPTY.
    while (i < n && ch[i]->label == EMPTY) ++i;
    return i == n;
  };

  if (!valid_arrangement()) {
    std::reverse(ch.begin(), ch.end());
    if (!valid_arrangement()) return false;
  }

  // Arrangement is now [non-EMPTY..., EMPTY...] with FULL at front.
  // Rebuild children, absorbing partial Q-children inline.
  // Pattern: [FULL*, PARTIAL?, FULL*, PARTIAL?, EMPTY*]
  std::vector<Node*> new_ch;

  int i = 0;
  // Left FULL run.
  while (i < n && ch[i]->label == FULL) { new_ch.push_back(ch[i]); ++i; }
  // Optional first PARTIAL (absorb, FULL end first).
  if (i < n && ch[i]->label == PARTIAL) {
    Node* pc = ch[i]; ++i;
    if (pc->type == Q_NODE) {
      GatherQChildren(pc);  // FULL at front
      for (Node* c : pc->children) { c->parent = node; new_ch.push_back(c); }
    } else {
      new_ch.push_back(pc);
    }
  }
  // Middle FULL run (between two partials).
  while (i < n && ch[i]->label == FULL) { new_ch.push_back(ch[i]); ++i; }
  // Optional second PARTIAL (absorb, reversed so FULL merges left).
  if (i < n && ch[i]->label == PARTIAL) {
    Node* pc = ch[i]; ++i;
    if (pc->type == Q_NODE) {
      GatherQChildren(pc);  // FULL at front
      // Reverse so FULL end merges with the full block to our left.
      std::vector<Node*> rev(pc->children.rbegin(), pc->children.rend());
      for (Node* c : rev) { c->parent = node; new_ch.push_back(c); }
    } else {
      new_ch.push_back(pc);
    }
  }
  // EMPTY run.
  while (i < n && ch[i]->label == EMPTY) { new_ch.push_back(ch[i]); ++i; }

  node->label = (CountFull(node) == static_cast<int>(node->children.size()))
                    ? FULL : PARTIAL;
  return true;
}

// ---------------------------------------------------------------------------
// AddConstraint with rollback on failure.
// ---------------------------------------------------------------------------
bool PQTree::AddConstraint(const std::vector<int>& subset) {
  if (!root_) return false;
  if (static_cast<int>(subset.size()) <= 1) return true;
  if (subset.size() == leaves_.size()) return true;

  // Deep-copy for rollback.
  Node* backup_root = DeepCopy(root_);
  std::vector<Node*> backup_leaves(leaves_.size());
  {
    std::queue<Node*> q;
    q.push(backup_root);
    while (!q.empty()) {
      Node* nd = q.front(); q.pop();
      if (nd->type == LEAF && nd->leaf_id >= 0 &&
          nd->leaf_id < static_cast<int>(backup_leaves.size())) {
        backup_leaves[nd->leaf_id] = nd;
      }
      for (Node* c : nd->children) q.push(c);
    }
  }

  if (!Bubble(subset) || !Reduce(subset)) {
    // Rollback.
    for (Node* nd : all_nodes_) delete nd;
    all_nodes_.clear();
    root_ = backup_root;
    leaves_ = backup_leaves;
    std::queue<Node*> q;
    q.push(root_);
    while (!q.empty()) {
      Node* nd = q.front(); q.pop();
      all_nodes_.push_back(nd);
      for (Node* c : nd->children) q.push(c);
    }
    return false;
  }

  // Success: free backup.
  FreeTree(backup_root);

  // Update leaves_ pointers (tree structure may have changed).
  for (size_t li = 0; li < leaves_.size(); ++li) leaves_[li] = nullptr;
  std::queue<Node*> q;
  q.push(root_);
  while (!q.empty()) {
    Node* nd = q.front(); q.pop();
    if (nd->type == LEAF && nd->leaf_id >= 0 &&
        nd->leaf_id < static_cast<int>(leaves_.size())) {
      leaves_[nd->leaf_id] = nd;
    }
    for (Node* c : nd->children) q.push(c);
  }

  return true;
}

int PQTree::AddConstraintsBestEffort(
    const std::vector<std::vector<int>>& subsets) {
  int count = 0;
  for (const auto& s : subsets) {
    if (AddConstraint(s)) ++count;
  }
  return count;
}

}  // namespace conex
