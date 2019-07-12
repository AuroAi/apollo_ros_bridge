load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_binary(
    name = "cyber_ros_bridge",
    srcs = ["cyber_ros_bridge_node.cpp"],
    deps = [
        "//cyber_ros_bridge/core:cyber_ros_bridge_core_lib"
    ],
)


cpplint()
