# Description:
#   Eigen is a C++ template library for linear algebra: vectors,
#   matrices, and related algorithms.

load("@rules_cc//cc:defs.bzl", "cc_library")

licenses([
    # Note: Eigen is an MPL2 library that includes GPL v3 and LGPL v2.1+ code.
    #       We've taken special care to not reference any restricted code.
    "reciprocal",  # MPL2
    "notice",  # Portions BSD
])

exports_files(["COPYING.MPL2"])

# Notable transitive dependencies of restricted files inside Eigen/...
EIGEN_RESTRICTED_DEPS = [
    "Eigen/Eigen",
    "Eigen/MetisSupport",
]

EIGEN_FILES = [
    "Eigen/**",
    "unsupported/Eigen/CXX11/**",
    "unsupported/Eigen/FFT",
    "unsupported/Eigen/KroneckerProduct",
    "unsupported/Eigen/src/FFT/**",
    "unsupported/Eigen/src/KroneckerProduct/**",
    "unsupported/Eigen/MatrixFunctions",
    "unsupported/Eigen/SpecialFunctions",
    "unsupported/Eigen/src/MatrixFunctions/**",
    "unsupported/Eigen/src/SpecialFunctions/**",
]

# List of files picked up by glob but actually part of another target.
EIGEN_EXCLUDE_FILES = [
    "Eigen/src/Core/arch/AVX/PacketMathGoogleTest.cc",
]

# Files known to be under MPL2 license.
EIGEN_MPL2_HEADER_FILES = glob(
    EIGEN_FILES,
    exclude = EIGEN_RESTRICTED_DEPS + [
        "Eigen/**/CMakeLists.txt",
    ],
)

cc_library(
    name = "eigen",
    hdrs = EIGEN_MPL2_HEADER_FILES,
    defines = [
        # This define (mostly) guarantees we don't link any problematic
        # code. We use it, but we do not rely on it, as evidenced above.
        "EIGEN_MPL2_ONLY",
        "EIGEN_MAX_ALIGN_BYTES=64",
        "EIGEN_HAS_TYPE_TRAITS=0",
    ],
    includes = ["."],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "eigen_header_files",
    srcs = EIGEN_MPL2_HEADER_FILES,
    visibility = ["//visibility:public"],
)
