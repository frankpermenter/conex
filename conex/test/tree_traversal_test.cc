#include <stdio.h>
#include <omp.h>
#include <Eigen/Dense>
#include <vector>

#include "traverse.h"

using Eigen::MatrixXd;
using std::vector;
using conex::RootedTree;
void test() {
 #pragma omp parallel num_threads(3)
 {
   // This code will be executed by three threads.
   
   // Chunks of this loop will be divided amongst
   // the (three) threads of the current team.
   #pragma omp for
   for(int n=0; n<10; ++n) printf(" %d", n);
 }
}

// 


std::vector<Eigen::LLT<MatrixXd>> FactorInParallel(const std::vector<MatrixXd>& matrices) {
bool parallelism_enabled = false;
int N = matrices.size();
std::vector<Eigen::LLT<MatrixXd>> llts(N);
 #pragma omp parallel for if(parallelism_enabled)
 for(int n = 0; n < N; ++n) { 
//   printf("%d, %f \n", n, matrices.at(n).trace());
   //printf("%d, \n", n);
   llts.at(n).compute(matrices.at(n));
 }
 return llts;
}


MatrixXd RandomPSD(int n) {
  MatrixXd X = MatrixXd::Random(n, n);
  return X * X.transpose();
}

void DoFactorInParallel() {
  int N = 4;
  int order = 40;
  vector<MatrixXd> matrices(N);
  for (int i = 0; i < N; i++) {
    matrices.at(i) = RandomPSD(order); 
  }
  FactorInParallel(matrices);
}

int main(int argc, char** argv){
//  DoFactorInParallel();
 
  RootedTree tree(3);
  tree.parent = vector<int>{-1, 0, 0};
  tree.height = vector<int>{0, 1, 1};
  conex::TraverseFromLeafs(tree);

  tree.parent = vector<int>{-1, 0, 1, -1};
  tree.height = vector<int>{0, 1, 2,  0};
  conex::TraverseFromLeafs(tree);
  conex::TraverseFromRoot(tree);

  tree.parent = vector<int>{-1, 0, 1, 0, 3, 4, 5, 6};
  tree.height = vector<int>{0, 1, 2,  1, 2, 3, 4, 5};
  conex::TraverseFromLeafs(tree);

  DUMP("HEHEHE");
  conex::TraverseFromRoot(tree);
  return 0;
}
