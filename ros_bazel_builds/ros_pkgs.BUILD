package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ros_pkgs",
    srcs = [
    ],
    hdrs = glob([
        "include/*/*.h",
    ]),
    include_prefix = "ros_pkgs",
    includes = ["include"],
    linkopts = [
        "-lrt",
        "-lboost_system",
    ],
)

