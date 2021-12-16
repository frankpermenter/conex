COPTS = [
    #"-DEIGEN_NO_DEBUG",
    #"-DNDEBUG",
    #"-O3",
    #    "-ffast-math",
]

cc_library(
    name = "test_util",
    srcs = ["test/test_util.cc"],
    hdrs = [
        "debug_macros.h",
        "test/test_util.h",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "conex_lib",
        "@gtest//:main",
    ],
)

cc_test(
    name = "lp",
    srcs = ["test/test_lp.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "core",
        "linear_constraint",
        "supernodal_solver",
        "@gtest//:main",
    ],
)

cc_test(
    name = "hermitian_psd_test",
    srcs = ["test/hermitian_psd_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "conex_lib",
        "test_util",
        "@gtest//:main",
    ],
)

cc_test(
    name = "sdp",
    srcs = ["test/test_sdp.cc"],
    copts = ["-Iexternal/gtest/include -fno-omit-frame-pointer"],
    deps = [
        "linear_constraint",
        "semidefinite_constraints",
        "test_util",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "psdp",
    srcs = ["test/profile_sdp.cc"],
    deps = [
        "conex_lib",
        "test_util",
        "@gtest//:main",
    ],
)

cc_test(
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
    copts = ["-Iexternal/gtest/include -O0"],
    deps = [
        "divergence",
        "@gtest//:main",
    ],
)

cc_test(
    name = "warmstart",
    srcs = ["test/test_warmstart.cc"],
    copts = ["-Iexternal/gtest/include -O0"],
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
    copts = COPTS,
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
    copts = COPTS,
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
    copts = COPTS,
    visibility = ["//visibility:public"],
    deps = [
        "@eigen",
    ],
)

cc_test(
    name = "jordan_matrix_algebra_test",
    srcs = ["test/jordan_matrix_algebra_test.cc"],
    copts = ["-Iexternal/gtest/include -O0"],
    deps = [
        "jordan_matrix_algebra",
        "test_util",
        "@gtest//:main",
    ],
)

cc_test(
    name = "exponential_map_test",
    srcs = ["test/exponential_map_test.cc"],
    copts = ["-Iexternal/gtest/include -O0"],
    deps = [
        "exponential_map",
        "@gtest//:main",
    ],
)

cc_library(
    name = "exponential_map",
    srcs = ["exponential_map.cc"],
    hdrs = ["exponential_map.h"],
    copts = COPTS,
    visibility = ["//visibility:public"],
    deps = [
        "jordan_matrix_algebra",
        "@eigen",
    ],
)

cc_test(
    name = "supernodal_solver_test",
    srcs = ["test/supernodal_solver_test.cc"],
    copts = ["-Iexternal/gtest/include -O0"],
    deps = [
        "supernodal_solver",
        "@gtest//:main",
    ],
)

cc_test(
    name = "block_triangular_operations_test",
    srcs = ["test/block_triangular_operations_test.cc"],
    copts = ["-Iexternal/gtest/include -O0"],
    deps = [
        "block_triangular_matrix",
        "@gtest//:main",
    ],
)

cc_library(
    name = "clique_ordering",
    srcs = [
        "clique_ordering.cc",
        "debug_macros.h",
    ],
    hdrs = [
        "clique_ordering.h",
    ],
    copts = COPTS,
    visibility = ["//visibility:public"],
    deps = [
        "tree_utils",
        "@eigen",
    ],
)

cc_test(
    name = "clique_ordering_test",
    srcs = ["test/clique_ordering_test.cc"],
    copts = ["-Iexternal/gtest/include -O0"],
    deps = [
        "clique_ordering",
        "@gtest//:main",
    ],
)

cc_test(
    name = "assembly_test",
    srcs = ["test/assembly_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "supernodal_solver",
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
    copts = COPTS,
    visibility = ["//visibility:public"],
    deps = ["@eigen"],
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
    name = "block_triangular_matrix",
    srcs = [
        "RLDLT.h",
        "block_triangular_operations.cc",
        "memory_utils.h",
        "parallel_block_triangular_operations.cc",
        "tree_traversal.h",
        "triangular_matrix_workspace.cc",
    ],
    hdrs = [
        "block_triangular_operations.h",
        "triangular_matrix_workspace.h",
    ],
    copts = COPTS + ["-fopenmp"],
    linkopts = ["-fopenmp"],
    deps = ["tree_utils"],
)

cc_library(
    name = "simple_triangular_matrix",
    srcs = [
        "tree_traversal.h",
        "dense_triangular_factorizations.h",
        "simple_triangular_matrix.cc",
    ],
    hdrs = [
        "simple_triangular_matrix.h",
    ],
    copts = ["-fopenmp"],
    deps = ["tree_utils"],
    visibility = ["//visibility:public"],
)


#cc_test(
#    name = "triangular_matrix_star_test",
#    srcs = [
#        "test/triangular_matrix_star_test.cc",
#    ],
#    copts = [
#        "-Iexternal/gtest/include",
#        "-g",
#    ],
#    deps = [
#        "simple_triangular_matrix",
#        "@gtest//:main",
#    ],
#)

cc_test(
    name = "simple_triangular_matrix_test",
    srcs = [
        "test/simple_triangular_matrix_test.cc",
    ],
    copts = [ "-Iexternal/gtest/include", "-fopenmp"],
    linkopts = ["-fopenmp"],
    deps = [
        "simple_triangular_matrix",
        "@gtest//:main",
    ],
)




cc_library(
    name = "supernodal_solver",
    srcs = [
        "debug_macros.h",
        "equality_constraint.cc",
        "kkt_solver.cc",
        "supernodal_assembler.cc",
        "supernodal_solver.cc",
    ],
    hdrs = [
        "constraint.h",
        "constraint_manager.h",
        "debug_macros.h",
        "equality_constraint.h",
        "error_checking_macros.h",
        "error_codes.h",
        "kkt_solver.h",
        "linear_workspace.h",
        "memory_utils.h",
        "newton_step.h",
        "supernodal_assembler.h",
        "supernodal_cholesky_data.h",
        "supernodal_solver.h",
        "workspace.h",
    ],
    copts = COPTS,
    visibility = ["//visibility:public"],
    deps = [
        "block_triangular_matrix",
        "clique_ordering",
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
        "constraint_manager.h",
        "debug_macros.h",
        "error_checking_macros.h",
        "error_codes.h",
        "memory_utils.h",
        "newton_step.h",
        "quadratic_cost.h",
        "supernodal_assembler.h",
        "supernodal_cholesky_data.h",
        "workspace.h",
    ],
    copts = COPTS,
    visibility = ["//visibility:public"],
    deps = [
        "divergence",
        ":supernodal_solver",
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
    copts = COPTS,
    visibility = ["//visibility:public"],
    deps = [
        "core",
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
    copts = COPTS,
    visibility = ["//visibility:public"],
    deps = [
        "approximate_eigenvalues",
        "core",
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
    copts = COPTS,
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
    copts = COPTS,
    visibility = ["//visibility:public"],
    deps = [
        "core",
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
        "newton_step.h",
        "workspace.h",
    ],
    copts = COPTS,
    linkopts = ["-lm"],
    visibility = ["//visibility:public"],
    deps = [
        "approximate_eigenvalues",
        "core",
        "divergence",
        "exponential_map",
        "exponential_map_pade",
        "jordan_matrix_algebra",
        "linear_constraint",
        "semidefinite_constraints",
        "socp_constraints",
        "supernodal_solver",
        "@eigen",
    ],
)

cc_test(
    name = "equality_constraints_test",
    srcs = ["test/equality_constraints_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "conex_lib",
        "@gtest//:main",
    ],
)

cc_test(
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
    srcs = ["test/kkt_solver_options_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "conex_lib",
        "@gtest//:main",
    ],
)


cc_binary(
    name = "tree_traversal_test",
    srcs = [
        "test/tree_traversal_test.cc",
        "tree_traversal.cc",
        "tree_traversal.h",
    ],
    copts = [
        "-g",
        "-fopenmp",
    ],
    linkopts = ["-fopenmp"],
    deps = [
        "tree_utils",
    ],
)

cc_binary(
    name = "solver_failures",
    srcs = ["test/solver_failures.cc"],
    copts = ["-g"],
    deps = [
        "conex_lib",
    ],
)
