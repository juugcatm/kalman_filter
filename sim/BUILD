cc_library (
    name = "simulation",
    hdrs = [
        "event.h",
	"generator.h",
        "scenario.h",
        "simulation.h",
        "time.h",
    ],
    srcs = [],
    deps = [
        "//external:glog",
    ],
)

cc_test (
    name = "simulation_test",
    srcs = ["simulation_test.cpp"],
    deps = [
        ":simulation",
        "//external:gtest_main",
    ],
)
