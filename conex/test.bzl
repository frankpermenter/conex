def conex_cc_test(
        name,
        size = None,
        srcs = [],
        args = [],
        tags = [],
        deps = [],
        copts = [],
        **kwargs):
    native.cc_test(
        name = name + "_cg",
        size = size,
        srcs = srcs + ["test/default_solver_config.h"],
        args = args,
        tags = tags,
        deps = deps,
        copts = copts + ["-DCONEX_TEST_CG"],
        **kwargs
    )
    native.cc_test(
        name = name,
        size = size,
        srcs = srcs + ["test/default_solver_config.h"],
        args = args,
        tags = tags,
        deps = deps,
        copts = copts,
        **kwargs
    )
