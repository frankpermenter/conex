#include <cstdio>
#include <string>
#include "conex/common/qps_reader.h"
using namespace conex;

void TestFile(const std::string& path) {
  printf("--- %s ---\n", path.c_str());
  try {
    auto [prob, info] = ReadQPS(path);
    printf("  vars=%d, eq_rows=%d, ineq_rows=%d, quad=%d, bounds=%d\n",
           info.num_variables, info.num_equality_rows,
           info.num_inequality_rows, info.num_quadratic_entries,
           info.num_bounded_vars);
    printf("  num_constraints=%d, num_variables=%d\n",
           prob.num_constraints(), prob.num_variables());
    if (prob.has_linear_cost())
      printf("  cost_size=%d\n", (int)prob.linear_cost().size());
  } catch (const std::exception& e) {
    printf("  ERROR: %s\n", e.what());
  }
}

int main() {
  const std::string dir = "/agent-workspace/problem_libraries/maros_meszaros/QPS_Files/";

  TestFile(dir + "QAFIRO.QPS");    // small, E+L rows, QUADOBJ
  TestFile(dir + "CVXQP1_S.QPS");  // all E rows, bounds, QUADOBJ
  TestFile(dir + "HS51.QPS");      // FR bounds
  TestFile(dir + "QADLITTL.QPS");  // E+L+G rows

  printf("\nDONE\n");
}
