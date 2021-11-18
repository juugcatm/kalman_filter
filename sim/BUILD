cc_library (
    name = "simulation"
    hdrs = [
        "event.h",
        "scenario.h",
        "simulation.h",
        "time.h",
    ],
    srcs = [],
)

cc_test (
    name = "simulation_test",
    srcs = ["simulation_test.cpp"],
    deps = [
        ":simulation",
        "//external:gtest_main",
    ],
)
