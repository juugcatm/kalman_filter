cc_library(
    name="kalman_filter",
    hdrs=["kalman_filter.h"],
)

cc_test(
    name="kalman_filter_test",
    srcs=["kalman_filter_test.cpp"],
    deps=[
        ":kalman_filter",
        "//external:glog",
        "//external:gtest_main",
    ]
)
