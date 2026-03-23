#include "conex/clique_ordering_utils.h"

#include <algorithm>

namespace conex {

void IntersectionOfSorted(const std::vector<int>& v1,
                          const std::vector<int>& v2, std::vector<int>* v3) {
  v3->clear();
  std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(),
                        back_inserter(*v3));
}

void Sort(std::vector<Clique>* path) {
  for (size_t i = 0; i < path->size(); i++) {
    std::sort(path->at(i).begin(), path->at(i).end());
  }
}

std::vector<int> UnionOfSorted(const std::vector<int>& x1,
                               const std::vector<int>& x2) {
  std::vector<int> y;
  set_union(x1.begin(), x1.end(), x2.begin(), x2.end(), inserter(y, y.end()));
  return y;
}

}  // namespace conex
