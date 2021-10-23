cc_library(
    name="kalman_filter",
    hdrs=["kalman_filter.h"],
    deps=[
        "//external:eigen",
    ]
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
