cc_library(
    name = "kalman_filter",
    hdrs = ["kalman_filter.h"],
    deps = [
        "//external:eigen",
    ],
    visibility = ["//visibility:public"]
)

cc_library(
    name = "particle_motion_filter",
    hdrs = ["particle_motion_filter.h"],
    deps = [
        ":kalman_filter",
        ":types",
        "//external:eigen",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "particle_motion_filter_test",
    srcs = ["particle_motion_filter_test.cpp"],
    deps = [
        ":particle_motion_filter",
        "//external:gtest_main",
        "//external:glog",
    ],
)

cc_library(
    name = "types",
    hdrs = ["types.h"],
)
