load("test.bzl", "conex_cc_test")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

cc_library(
    name = "test_util",
    srcs = ["test/test_util.cc"],
    hdrs = [
        "debug_macros.h",
        "test/test_util.h",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "@eigen",
        "@gtest//:main",
    ],
)

conex_cc_test(
    name = "linear_constraint_test",
    srcs = ["test/linear_constraint_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "core",
        "linear_constraint",
        "@gtest//:main",
    ],
)

conex_cc_test(
    name = "hermitian_psd_test",
    srcs = ["test/hermitian_psd_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "core",
        "semidefinite_constraints",
        "test_util",
        "@gtest//:main",
    ],
)

conex_cc_test(
    name = "sdp_test",
    srcs = ["test/sdp_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "core",
        "linear_constraint",
        "semidefinite_constraints",
        "test_util",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "psdp",
    srcs = [
        "test/default_solver_config.h",
        "test/profile_sdp.cc",
    ],
    deps = [
        "conex_lib",
        "test_util",
        "@gtest//:main",
    ],
)

conex_cc_test(
    name = "socp",
    srcs = ["test/test_socp.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "conex_lib",
        "@gtest//:main",
    ],
)

cc_test(
    name = "divergence_test",
    srcs = ["test/test_divergence.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "divergence",
        "@gtest//:main",
    ],
)

conex_cc_test(
    name = "warmstart",
    srcs = ["test/test_warmstart.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "conex_lib",
        "test_util",
        "@gtest//:main",
    ],
)

cc_test(
    name = "approximate_eigenvalues_test",
    srcs = [
        "debug_macros.h",
        "test/approximate_eigenvalues.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "approximate_eigenvalues",
        "conex_lib",
        "test_util",
        "@eigen",
        "@gtest//:main",
    ],
)

cc_library(
    name = "approximate_eigenvalues",
    srcs = ["approximate_eigenvalues.cc"],
    hdrs = ["approximate_eigenvalues.h"],
    visibility = ["//visibility:public"],
    deps = ["@eigen"],
)

cc_test(
    name = "exponential_map_pade_test",
    srcs = ["test/exponential_map_pade_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "exponential_map_pade",
        "@eigen",
        "@gtest//:main",
    ],
)

cc_library(
    name = "exponential_map_pade",
    srcs = [
        "debug_macros.h",
        "exponential_map_pade.cc",
    ],
    hdrs = ["exponential_map_pade.h"],
    visibility = ["//visibility:public"],
    deps = ["@eigen"],
)

cc_library(
    name = "jordan_matrix_algebra",
    srcs = [
        "debug_macros.h",
        "jordan_matrix_algebra.cc",
    ],
    hdrs = ["jordan_matrix_algebra.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@eigen",
    ],
)

cc_test(
    name = "jordan_matrix_algebra_test",
    srcs = ["test/jordan_matrix_algebra_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "jordan_matrix_algebra",
        "test_util",
        "@gtest//:main",
    ],
)

cc_test(
    name = "exponential_map_test",
    srcs = ["test/exponential_map_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "exponential_map",
        "@gtest//:main",
    ],
)

cc_library(
    name = "exponential_map",
    srcs = ["exponential_map.cc"],
    hdrs = ["exponential_map.h"],
    visibility = ["//visibility:public"],
    deps = [
        "jordan_matrix_algebra",
        "@eigen",
    ],
)

conex_cc_test(
    name = "supernodal_solver_test",
    srcs = ["test/supernodal_solver_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "supernodal_solver",
        "@gtest//:main",
    ],
)

cc_test(
    name = "block_triangular_operations_test",
    srcs = ["test/block_triangular_operations_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "supernodal_solver",
        "@gtest//:main",
    ],
)

cc_test(
    name = "clique_ordering_test",
    srcs = ["test/clique_ordering_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "clique_ordering",
        "@gtest//:main",
    ],
)

conex_cc_test(
    name = "assembly_test",
    srcs = ["test/assembly_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "kkt_solver_factory",
        "@gtest//:main",
    ],
)

cc_test(
    name = "tree_utils_test",
    srcs = ["test/tree_utils_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "tree_utils",
        "@gtest//:main",
    ],
)

cc_library(
    name = "tree_utils",
    srcs = [
        "debug_macros.h",
        "tree_utils.cc",
    ],
    hdrs = [
        "tree_utils.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "error_checking",
        "@eigen",
    ],
)

cc_library(
    name = "solver_configuration",
    hdrs = ["conex.h"],
)

cc_library(
    name = "quadratic_cost",
    srcs = ["quadratic_cost.cc"],
    hdrs = [
        "quadratic_cost.h",
        "workspace.h",
    ],
    deps = [
        "constraint_interface",
        "error_checking",
        "@eigen",
    ],
)

cc_library(
    name = "constraint_manager",
    srcs = ["constraint_manager.cc"],
    hdrs = [
        "conex.h",
        "constraint_manager.h",
    ],
    deps = [
        "clique_ordering",
        "equality_constraint",
        "error_checking",
        "kkt_solver_interface",
        "quadratic_cost",
    ],
)

cc_library(
    name = "kkt_solver_interface",
    hdrs = ["kkt_solver_interface.h"],
)

cc_library(
    name = "kkt_solver_factory",
    srcs = ["kkt_solver_factory.cc"],
    hdrs = ["kkt_solver_factory.h"],
    deps = [
        "conjugate_gradient_solvers",
        "kkt_solver_interface",
        "kkt_tree_solver",
        "supernodal_solver",
    ],
)

cc_library(
    name = "error_checking",
    hdrs = [
        "error_checking_macros.h",
        "error_codes.h",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "clique_ordering",
    srcs = [
        "clique_ordering.cc",
        "clique_ordering_md.cc",
        "clique_ordering_utils.cc",
    ],
    hdrs = [
        "clique_ordering.h",
        "clique_ordering_utils.h",
    ],
    deps = [
        "error_checking",
        "tree_utils",
    ],
)

cc_test(
    name = "clique_ordering_md_test",
    srcs = ["test/clique_ordering_md_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "clique_ordering",
        "@gtest//:main",
    ],
)

cc_library(
    name = "supernodal_solver",
    srcs = [
        "RLDLT.h",
        "block_triangular_operations.cc",
        "debug_macros.h",
        "kkt_solver.cc",
        "supernodal_assembler.cc",
        "supernodal_solver.cc",
        "triangular_matrix_workspace.cc",
    ],
    hdrs = [
        "block_triangular_operations.h",
        "constraint.h",
        "debug_macros.h",
        "kkt_solver.h",
        "kkt_solver_factory.h",
        "kkt_solver_interface.h",
        "linear_workspace.h",
        "memory_utils.h",
        "newton_step.h",
        "supernodal_assembler.h",
        "supernodal_cholesky_data.h",
        "supernodal_solver.h",
        "triangular_matrix_workspace.h",
        "workspace.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "equality_constraint",
        ":clique_ordering",
        ":constraint_manager",
        ":error_checking",
        ":solver_configuration",
        ":tree_utils",
        "@eigen",
    ],
)

cc_library(
    name = "core",
    srcs = [
        "cone_program.cc",
        "self_dual_embedding.cc",
    ],
    hdrs = [
        "cone_program.h",
        "constraint.h",
        "debug_macros.h",
        "memory_utils.h",
        "newton_step.h",
        "self_dual_embedding.h",
        "supernodal_cholesky_data.h",
        "workspace.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "constraint_interface",
        ":conjugate_gradient_solvers",
        ":constraint_manager",
        ":divergence",
        ":error_checking",
        ":kkt_solver_factory",
        ":serializer",
        ":solver_configuration",
        ":supernodal_solver",
        "@eigen",
    ],
)

cc_library(
    name = "equality_constraint",
    srcs = [
        "equality_constraint.cc",
    ],
    hdrs = [
        "equality_constraint.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "constraint_interface",
        "@eigen",
    ],
)

cc_library(
    name = "linear_constraint",
    srcs = [
        "linear_constraint.cc",
    ],
    hdrs = [
        "linear_constraint.h",
        "linear_workspace.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "constraint_interface",
        "error_checking",
        "solver_configuration",
        "@eigen",
    ],
)

cc_library(
    name = "semidefinite_constraints",
    srcs = [
        "dense_lmi_constraint.cc",
        "hermitian_psd.cc",
        "psd_constraint.cc",
    ],
    hdrs = [
        "dense_lmi_constraint.h",
        "hermitian_psd.h",
        "psd_constraint.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "approximate_eigenvalues",
        "constraint_interface",
        "error_checking",
        "exponential_map",
        "exponential_map_pade",
        "jordan_matrix_algebra",
        "solver_configuration",
        "@eigen",
    ],
)

cc_library(
    name = "divergence",
    srcs = [
        "divergence.cc",
    ],
    hdrs = [
        "debug_macros.h",
        "divergence.h",
        "memory_utils.h",
        "newton_step.h",
    ],
    visibility = ["//visibility:public"],
    deps = ["@eigen"],
)

cc_library(
    name = "socp_constraints",
    srcs = [
        "quadratic_cone_constraint.cc",
        "quadratic_epigraph.cc",
        "soc_constraint.cc",
        "workspace_soc.h",
    ],
    hdrs = [
        "quadratic_cone_constraint.h",
        "soc_constraint.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "constraint_interface",
        "error_checking",
        "@eigen",
    ],
)

cc_library(
    name = "conex_lib",
    hdrs = [
        "cone_program.h",
        "constraint.h",
        "debug_macros.h",
        "memory_utils.h",
        "newton_step.h",
        "workspace.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":approximate_eigenvalues",
        ":core",
        ":divergence",
        ":equality_constraint",
        ":error_checking",
        ":exponential_map",
        ":exponential_map_pade",
        ":jordan_matrix_algebra",
        ":linear_constraint",
        ":semidefinite_constraints",
        ":socp_constraints",
        ":solver_configuration",
        ":supernodal_solver",
        "@eigen",
    ],
)

conex_cc_test(
    name = "equality_constraints_test",
    srcs = [
        "test/equality_constraints_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        ":core",
        ":equality_constraint",
        ":linear_constraint",
        "@gtest//:main",
    ],
)

conex_cc_test(
    name = "quadratic_objective_test",
    srcs = ["test/quadratic_objective_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "conex_lib",
        "@gtest//:main",
    ],
)

cc_test(
    name = "kkt_solver_options_test",
    srcs = [
        "test/default_solver_config.h",
        "test/kkt_solver_options_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "conex_lib",
        "@gtest//:main",
    ],
)

cc_test(
    name = "kkt_tree_solver_speedup_test",
    srcs = [
        "test/kkt_tree_solver_speedup_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "kkt_tree_solver",
        "@gtest//:main",
    ],
)

cc_test(
    name = "kkt_tree_solver_sparse_benchmark_test",
    srcs = [
        "test/kkt_tree_solver_sparse_benchmark_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "kkt_tree_solver",
        "@gtest//:main",
    ],
)

cc_test(
    name = "kkt_tree_solver_least_squares_test",
    srcs = [
        "test/kkt_tree_solver_least_squares_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "clique_ordering",
        "kkt_tree_solver",
        "@gtest//:main",
    ],
)

cc_test(
    name = "kkt_tree_solver_generic_factorization_test",
    srcs = [
        "test/kkt_tree_solver_generic_factorization_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "kkt_tree_solver",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "kkt_tree_solver_least_squares_benchmark",
    srcs = [
        "test/kkt_tree_solver_least_squares_benchmark.cc",
    ],
    deps = [
        "clique_ordering",
        "kkt_tree_solver",
        "@eigen",
    ],
)

cc_library(
    name = "conjugate_gradient_solvers",
    srcs = ["conjugate_gradient_solvers.cc"],
    hdrs = ["conjugate_gradient_solvers.h"],
    deps = [
        "supernodal_solver",
    ],
)

cc_test(
    name = "conjugate_gradient_solvers_test",
    srcs = ["test/conjugate_gradient_solvers_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "conjugate_gradient_solvers",
        "kkt_solver_factory",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "solver_failures",
    srcs = ["test/solver_failures.cc"],
    data = [
        "test/graph_of_convex_sets_fails_slater.json",
        "test/maze_example.json",
        "test/maze_example_2.json",
    ],
    deps = [
        "conex_lib",
        "self_dual_embedding",
        "serializer",
    ],
)

cc_library(
    name = "constraint_interface",
    hdrs = [
        "constraint.h",
        "constraint_interface.h",
        "debug_macros.h",
        "memory_utils.h",
        "newton_step.h",
        "supernodal_assembler_base.h",
        "supernodal_cholesky_data.h",
        "workspace.h",
    ],
    deps = [
        "error_checking",
    ],
)

cc_library(
    name = "serializer",
    srcs = [
        "json_parser.cc",
        "json_parser.h",
        "serialize.cc",
        "serialize.h",
    ],
    deps = [
        "constraint_manager",
        "equality_constraint",
        "error_checking",
        "linear_constraint",
        "semidefinite_constraints",
        "socp_constraints",
        "@eigen",
    ],
)

cc_test(
    name = "serialize_test",
    srcs = [
        "test/serialize_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "core",
        "equality_constraint",
        "linear_constraint",
        "semidefinite_constraints",
        "serializer",
        "socp_constraints",
        "@eigen",
        "@gtest//:main",
    ],
)

cc_test(
    name = "constraint_visibility_test",
    srcs = [
        "test/constraint_visibility_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    data = [
        "constraint.h",
        "dense_lmi_constraint.h",
        "equality_constraint.h",
        "hermitian_psd.h",
        "linear_constraint.h",
        "psd_constraint.h",
        "quadratic_cone_constraint.h",
        "soc_constraint.h",
    ],
    deps = [
        "@gtest//:main",
    ],
)

cc_library(
    name = "kkt_tree_solver",
    srcs = [
        "RLDLT.h",
        "cholesky_solvers.h",
        "debug_macros.h",
        "kkt_simplicial_solver.cc",
        "kkt_simplicial_solver.h",
        "kkt_subsystem.cc",
        "kkt_subsystem.h",
        "kkt_tree_solver.cc",
        "kkt_tree_solver.h",
        "static_subsystem.cc",
        "static_subsystem.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "constraint_interface",
        "tree_utils",
        ":error_checking",
        ":kkt_solver_interface",
        "@eigen",
    ],
)

cc_test(
    name = "self_dual_embedding_test",
    srcs = ["test/self_dual_embedding_test.cc"],
    copts = [
        "-Iexternal/gtest/include",
        "-g",
    ],
    deps = [
        "conex_lib",
        "self_dual_embedding",
        "test_util",
        "@gtest//:main",
    ],
)

cc_library(
    name = "self_dual_embedding",
    srcs = [
        "self_dual_embedding.cc",
    ],
    hdrs = [
        "self_dual_embedding.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "conex_lib",
        "@eigen",
    ],
)
