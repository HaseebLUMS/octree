load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_pcl//bzl:pcl.bzl", "pcl_config")
pcl_config()

cc_binary(
    name = "benchmark",
    srcs = ["benchmark.cc"],
    deps = ["@pcl//:common", "@pcl//:octree", "@pcl//:io"]
)

cc_binary(
    name = "encdec",
    srcs = ["main.cc", "encode.hpp", "decode.hpp", "common.h", "common.cc"],
    deps = ["@pcl//:common", "@pcl//:octree", "@pcl//:io"],
    data = ["assets/ricardo.ply", "output"]
)