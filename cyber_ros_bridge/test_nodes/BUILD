load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:private"])


cc_binary(
    name = "yaml_generator",
    srcs = ["yaml_generator.cpp"],
    deps = [
        "@yaml_cpp//:yaml"
    ],
)


cc_binary(
    name = "yaml_reader",
    srcs = ["yaml_reader.cpp"],
    deps = [
        "@yaml_cpp//:yaml"
    ],
)


cc_binary(
    name = "parser_test",
    srcs = ["test_parselib.cpp"],
    deps = [
        "//cyber_ros_bridge/core:cyber_ros_bridge_core_lib"
    ],
)





cpplint()
