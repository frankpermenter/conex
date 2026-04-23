load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

cc_library(
    name = "libconex",
    srcs = ["conex.cc"],
    hdrs = ["conex.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//conex:conex_lib",
        "@eigen",
    ],
)

cc_test(
    name = "interface_test",
    srcs = ["test/interface_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "libconex",
        "@gtest//:main",
    ],
)

cc_test(
    name = "interface_test_lmi",
    srcs = ["test/interface_test_lmi.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "libconex",
        "@gtest//:main",
    ],
)

# The same tests but against .so built by the custom Makefile
cc_test(
    name = "interface_lib_test",
    srcs = [
        "conex.h",
        "test/interface_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "libconex",
        "@gtest//:main",
    ],
)

cc_test(
    name = "interface_lib_test_lmi",
    srcs = [
        "conex.h",
        "test/interface_test_lmi.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "libconex",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "test_app",
    srcs = [
        "conex.h",
        "test/test_app.cc",
    ],
    deps = ["libconex"],
)
