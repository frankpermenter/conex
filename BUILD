load("@rules_shell//shell:sh_binary.bzl", "sh_binary")

sh_binary(
    name = "buildifier",
    srcs = ["tools/run_buildifier.sh"],
    args = ["$(location @buildifier_linux_amd64//file)"],
    data = ["@buildifier_linux_amd64//file"],
)
