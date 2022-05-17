load("test.bzl", "conex_cc_test")

cc_library(
    name = "test_util",
    srcs = glob(["test/test_util.cc"]),
    hdrs = glob([
        "test/test_util.h",
        "debug_macros.h",
    ]),
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "conex_lib",
        "@gtest//:main",
    ],
)

conex_cc_test(
    name = "linear_constraint_test",
    srcs = glob(["test/linear_constraint_test.cc"]),
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "core",
        "linear_constraint",
        "@gtest//:main",
    ],
)

conex_cc_test(
    name = "hermitian_psd_test",
    srcs = glob(["test/hermitian_psd_test.cc"]),
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "conex_lib",
        "test_util",
        "@gtest//:main",
    ],
)

conex_cc_test(
    name = "sdp",
    srcs = glob(["test/test_sdp.cc"]),
    copts = ["-Iexternal/gtest/include"],
    deps = [
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
    srcs = glob(["test/test_socp.cc"]),
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "conex_lib",
        "@gtest//:main",
    ],
)

cc_test(
    name = "divergence_test",
    srcs = glob(["test/test_divergence.cc"]),
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
    srcs = glob([
        "test/approximate_eigenvalues.cc",
        "debug_macros.h",
    ]),
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
    srcs = glob(["test/exponential_map_pade_test.cc"]),
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
        "supernodal_solver",
        "@gtest//:main",
    ],
)

conex_cc_test(
    name = "assembly_test",
    srcs = [
        "test/assembly_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "constraint_interface",
        "kkt_solver_interface",
        "supernodal_solver",
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
        ":error_checking",
        "@eigen",
    ],
)

cc_library(
    name = "solver_configuration",
    hdrs = ["conex.h"],
)

cc_library(
    name = "kkt_solver_interface",
    hdrs = [
        "constraint_manager.h",
        "kkt_solver_interface.h",
        "supernodal_assembler_base.h",
        "supernodal_cholesky_data.h",
        "newton_step.h",
        "memory_utils.h",
    ],
)

cc_library(
    name = "kkt_solver_factory",
    srcs = [
        "kkt_solver_factory.cc",
    ],
    hdrs = ["kkt_solver_factory.h"],
    deps = [
        "conjugate_gradient_solvers",
        "constraint_interface",
        "kkt_solver_interface",
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
    name = "supernodal_solver",
    srcs = [
        "RLDLT.h",
        "block_triangular_operations.cc",
        "clique_ordering.cc",
        "debug_macros.h",
        "kkt_solver.cc",
        "supernodal_assembler.cc",
        "supernodal_solver.cc",
        "triangular_matrix_workspace.cc",
    ],
    hdrs = [
        "block_triangular_operations.h",
        "clique_ordering.h",
        "debug_macros.h",
        "kkt_solver.h",
        "kkt_solver_factory.h",
        "kkt_solver_interface.h",
        "linear_workspace.h",
        "memory_utils.h",
        "supernodal_assembler.h",
        "supernodal_cholesky_data.h",
        "supernodal_solver.h",
        "triangular_matrix_workspace.h",
        "workspace.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "kkt_solver_interface",
        "constraint_interface",
        "equality_constraint",
        "error_checking",
        "solver_configuration",
        "tree_utils",
        "@eigen",
    ],
)

cc_library(
    name = "core",
    srcs = [
        "cone_program.cc",
        "quadratic_cost.cc",
    ],
    hdrs = [
        "cone_program.h",
        "constraint.h",
        "constraint_interface.h",
        "debug_macros.h",
        "memory_utils.h",
        "newton_step.h",
        "quadratic_cost.h",
        "supernodal_assembler.h",
        "supernodal_cholesky_data.h",
        "workspace.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":conjugate_gradient_solvers",
        ":divergence",
        ":equality_constraint",
        ":error_checking",
        ":kkt_solver_factory",
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
        "error_checking_macros.h",
        "error_codes.h",
        "memory_utils.h",
        "supernodal_assembler.h",
        "workspace.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "approximate_eigenvalues",
        "constraint_interface",
        "core",
        "divergence",
        "equality_constraint",
        "exponential_map",
        "exponential_map_pade",
        "jordan_matrix_algebra",
        "linear_constraint",
        "semidefinite_constraints",
        "socp_constraints",
        "solver_configuration",
        "supernodal_solver",
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
    srcs = glob(["test/quadratic_objective_test.cc"]),
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
    deps = [
        "conex_lib",
    ],
)

cc_library(
    name = "constraint_interface",
    hdrs = [
        "constraint.h",
        "constraint_interface.h",
        "memory_utils.h",
        "newton_step.h",
        "supernodal_assembler.h",
        "supernodal_cholesky_data.h",
        "workspace.h",
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
        "equality_constraint",
        "linear_constraint",
        "semidefinite_constraints",
        "serializer",
        "socp_constraints",
        "@eigen",
        "@gtest//:main",
    ],
)

cc_library(
    name = "kkt_tree_solver",
    srcs = [
        "debug_macros.h",
        "kkt_subsystem.cc",
        "kkt_subsystem.h",
        "cholesky_solvers.h",
        "kkt_tree_solver.cc",
        "kkt_tree_solver.h",
        "static_subsystem.h",
        "static_subsystem.cc",
        "RLDLT.h",
    ],
    deps = [
        "tree_utils",
        ":error_checking",
        ":kkt_solver_interface",
        "@eigen",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "kkt_subsystem_test",
    srcs = [
        "RLDLT.h",
        "debug_macros.h",
        "test/kkt_subsystem_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "kkt_tree_solver",
        ":error_checking",
        ":kkt_solver_interface",
        "@eigen",
        "@gtest//:main",
    ],
)

cc_test(
    name = "kkt_solver_profile",
    srcs = [
        "RLDLT.h",
        "debug_macros.h",
        "test/kkt_solver_profile.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "kkt_tree_solver",
        ":error_checking",
        ":kkt_solver_interface",
        "@eigen",
        "@gtest//:main",
    ],
)
