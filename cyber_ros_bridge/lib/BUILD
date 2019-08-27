
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cyber_ros_bridge_lib",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.hpp"]),
   # includes = ["../common"],
    deps = [
        "//cyber",
        "@ros_indigo//:ros_indigo_common",
        "@yaml_cpp//:yaml",
        "@ros_pkgs//:ros_pkgs",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:pose_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/canbus/proto:canbus_proto",
        "//modules/control/proto:control_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/drivers/gnss/proto:imu_proto",
        "//cyber_ros_bridge/common:all_gflags"
    ],

    #visibility = ["//cyber_ros_bridge/core:__pkg__"],
)

