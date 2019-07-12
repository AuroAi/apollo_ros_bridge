load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "parser_lib",
    srcs = ["parse_yaml.cpp",],
    hdrs = ["parse_yaml.hpp",],
    deps = [
        "//cyber",
        "@ros_indigo//:ros_indigo_common",
        "@yaml_cpp//:yaml"

    ],
)


cc_library(
    name = "cyber_ros_bridge_core_lib",
    srcs = ["cyber_ros_bridge_core.cpp",],
    hdrs = ["cyber_ros_bridge_core.hpp"],
    #includes = ["../lib"],
    deps = [
        ":parser_lib",
        "//cyber_ros_bridge/lib:cyber_ros_bridge_lib"
    ],
)



cpplint()
