load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "quadratic_programs",
    srcs = ["quadratic_programs.cc"],
    copts = [
        "-O0",
        "-g",
    ],
    deps = [
        "//conex:conex_lib",
    ],
)
