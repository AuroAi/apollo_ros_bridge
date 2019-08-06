package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ros_indigo_common",
    srcs = [
        "lib/libcpp_common.so",
        "lib/librosbag.so",
        "lib/librosbag_storage.so",
        "lib/librosconsole.so",
        "lib/librosconsole_backend_interface.so",
        "lib/librosconsole_log4cxx.so",
        "lib/libroscpp.so",
        "lib/libroscpp_serialization.so",
        "lib/libroslz4.so",
        "lib/librostime.so",
        "lib/libtopic_tools.so",
        "lib/libxmlrpcpp.so",
        "lib/libmessage_filters.so",
        "lib/libroslib.so",
        "lib/librospack.so",
    ],
    hdrs = glob([
        "include/*/*.h",
    ]),
    include_prefix = "ros",
    includes = ["include"],
    linkopts = [
        "-lrt",
        "-lboost_system",
    ],
)

