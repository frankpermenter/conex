#include <vector>
namespace conex {

using Clique = std::vector<int>;
void Sort(std::vector<Clique>* path);

std::vector<int> UnionOfSorted(const std::vector<int>& x1,
                               const std::vector<int>& x2);
void IntersectionOfSorted(const std::vector<int>& v1,
                          const std::vector<int>& v2, std::vector<int>* v3);

}  // namespace conex
