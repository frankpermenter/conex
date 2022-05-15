set -e

DIR=/home/frank/conex/bazel-out/k8-dbg/bin/examples/graph_of_convex_sets/edge_topological_order_test.runfiles/conex
rm -rf $DIR 
mkdir $DIR
#mkdir $DIR/path
#mkdir $DIR/random
mkdir $DIR/nonunique
bazel run :edge_topological_order_test  --config=debug
python spy.py


#bazel run :supernode_submatrix_test  --config=debug

#bazel run :graph_of_convex_sets_kkt_solver_test  --config=benchmark

#DIR=/home/frank/conex/bazel-out/k8-dbg/bin/examples/graph_of_convex_sets/gcs_solver_test.runfiles/conex


#DIR=/home/frank/conex/bazel-out/k8-dbg/bin/examples/graph_of_convex_sets/gcs_solver_test.runfiles/conex
#rm -rf $DIR 
#mkdir $DIR
#mkdir $DIR/path
#mkdir $DIR/random
#mkdir $DIR/nonunique
#mkdir $DIR/simple
#
#bazel run :gcs_solver_test  --config=debug
#python spy.py


#../../bazel-bin/examples/graph_of_convex_sets/graph_of_convex_sets_kkt_solver_test 
#gprof ../../bazel-bin/examples/graph_of_convex_sets/graph_of_convex_sets_kkt_solver_test gmon.out > output.txt 

#bazel run :llt_profile  --config=debug

#bazel run :llt_profile  --config=benchmark
#../../bazel-bin/examples/graph_of_convex_sets/llt_profile
#gprof ../../bazel-bin/examples/graph_of_convex_sets/llt_profile gmon.out > output.txt 

#../../bazel-bin/examples/graph_of_convex_sets/graph_of_convex_sets_kkt_solver_test 
#gprof ../../bazel-bin/examples/graph_of_convex_sets/graph_of_convex_sets_kkt_solver_test gmon.out > output.txt 
