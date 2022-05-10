cc_library(name = "graph_of_convex_sets_data",
  hdrs = ["graph_data.h"],
)

cc_library(name = "graph_of_convex_sets",
  srcs = ["gcs_solver.cc"],
  hdrs = ["gcs_solver.h"],
  deps = [
      ":graph_of_convex_sets_data",
      ":graph_of_convex_sets_util",
      "//conex:error_checking",
      "//conex:kkt_tree_solver",
      "@gtest//:main",
  ],
)

cc_library(
    name = "graph_of_convex_sets_util",
    srcs = ["directed_graph.cc",
            "kkt_subsystem.cc",
            "supernode_inverse.cc", 
            "supernode_inverse.h", 
            "convex_set_node_factory.h",
            "convex_set_node_factory.cc",
            "directed_graph.h"],
    copts = ["-Iexternal/gtest/include"],
    hdrs = ["directed_graph.h", "kkt_subsystem.h"],
    deps = [
        ":graph_of_convex_sets_data",
        "//conex:error_checking",
        "//conex:kkt_tree_solver",
        "@gtest//:main",
    ],
)

cc_test(
  name = "graph_of_convex_sets_kkt_solver_test",
  srcs = ["graph_of_convex_sets_kkt_solver_test.cc", 
          "profile.cc",
          "profile.h"],
  copts = ["-Iexternal/gtest/include", 
           "-DEIGEN_RUNTIME_NO_MALLOC"],
  deps = [
      "graph_of_convex_sets",
      "graph_of_convex_sets_util",
      "//conex:error_checking",
      "//conex:kkt_tree_solver",
      "@gtest//:main",
  ],
)

cc_test(
  name = "llt_profile",
  srcs = ["llt_profile.cc"],
  copts = ["-Iexternal/gtest/include", 
           "-DEIGEN_RUNTIME_NO_MALLOC"],
  deps = [
      "//conex:error_checking",
      "//conex:kkt_tree_solver",
      "@gtest//:main",
  ],
)

cc_test(
    name = "supernode_submatrix_test",
    srcs = [ "supernode_inverse_test.cc", 
    "supernode_inverse.cc", "supernode_inverse.h", 
    "kkt_subsystem.h"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//conex:error_checking",
        "//conex:kkt_tree_solver",
        "@gtest//:main",
    ],
)

cc_test(
    name = "fill_in_comparison_test",
    srcs = [ "fill_in_comparison_test.cc", "profile.h", "profile.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "graph_of_convex_sets",
        "//conex:error_checking",
        "//conex:kkt_tree_solver",
        "@gtest//:main",
    ],
)
