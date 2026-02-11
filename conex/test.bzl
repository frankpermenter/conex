load("@rules_cc//cc:defs.bzl", "cc_test")

def conex_cc_test(
        name,
        size = None,
        srcs = [],
        args = [],
        tags = [],
        deps = [],
        copts = [],
        **kwargs):
    cc_test(
        name = name + "_sd_embedding",
        size = size,
        srcs = srcs + ["test/default_solver_config.h"],
        args = args,
        tags = tags,
        deps = deps,
        copts = copts + ["-DCONEX_TEST_SD_EMBEDDING"],
        **kwargs
    )

    cc_test(
        name = name + "_cg",
        size = size,
        srcs = srcs + ["test/default_solver_config.h"],
        args = args,
        tags = tags,
        deps = deps,
        copts = copts + ["-DCONEX_TEST_CG"],
        **kwargs
    )

    cc_test(
        name = name + "_tree",
        size = size,
        srcs = srcs + ["test/default_solver_config.h"],
        args = args,
        tags = tags,
        deps = deps,
        copts = copts + ["-DCONEX_TEST_TREE"],
        **kwargs
    )

    cc_test(
        name = name,
        size = size,
        srcs = srcs + ["test/default_solver_config.h"],
        args = args,
        tags = tags,
        deps = deps,
        copts = copts + ["-DCONEX_TEST_SUPERNODAL"],
        **kwargs
    )
