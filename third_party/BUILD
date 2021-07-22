# -*- python -*-
# Copyright 2016-2019 Toyota Research Institute.  All rights reserved.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mkl",
    srcs = [
        #        "mkl/lib/intel64/libmkl_core.a",
        #        "mkl/lib/intel64/libmkl_gnu_thread.a",
        #        "mkl/lib/intel64/libmkl_intel_lp64.a",
    ],
    hdrs = glob(["mkl/include/*.h"]),
    copts = ["-Imkl/include"],
    linkopts = [
        "-lpthread",
        "-lm",
        "-ldl",
        "-fopenmp",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "eigen",
    deps = [
        "mkl",
        "@eigen",
    ],
)
