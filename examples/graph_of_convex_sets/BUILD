cc_test(
    name = "graph_of_convex_sets_kkt_solver_test",
    srcs = ["graph_of_convex_sets_kkt_solver_test.cc", 
            "directed_graph.cc",
            "kkt_subsystem.h",
            "kkt_subsystem.cc",
            "profile.cc",
            "profile.h",
            "convex_set_node_factory.h",
            "convex_set_node_factory.cc",
            "directed_graph.h"],
    copts = ["-Iexternal/gtest/include", 
             "-DEIGEN_RUNTIME_NO_MALLOC"],
    #outs = ["gmon.out"],
    deps = [
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
    #outs = ["gmon.out"],
    deps = [
        "//conex:error_checking",
        "//conex:kkt_tree_solver",
        "@gtest//:main",
    ],
)
