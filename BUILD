load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_pcl//bzl:pcl.bzl", "pcl_config")
pcl_config()

cc_library(
    name = "jpegencdec",
    srcs = [
        "libs/JpegDecoder.cpp",
        "libs/JpegDecoder.hpp",
        "libs/JpegEncoder.cpp",
        "libs/JpegEncoder.hpp",
        "libs/jpeg-turbo/2.1.5.1/lib/libturbojpeg.a",
        "libs/jpeg-turbo/2.1.5.1/include/turbojpeg.h",
    ],
    visibility = ["//main:__pkg__"],
)

cc_binary(
    name = "benchmark",
    srcs = ["benchmark.cc"],
    deps = ["@pcl//:common", "@pcl//:octree", "@pcl//:io", "//:jpegencdec"]
)

cc_binary(
    name = "encdec",
    srcs = ["main.cc", "encode.hpp", "decode.hpp", "common.h", "common.cc"],
    deps = ["@pcl//:common", "@pcl//:octree", "@pcl//:io", "//:jpegencdec"],
    data = ["assets", "output"]
)

cc_binary(
    name = "server",
    srcs = ["server.cc", "config.h"],
    linkopts = ["-lpthread"],
)

cc_binary(
    name = "client",
    srcs = ["client.cc", "config.h"],
    linkopts = ["-lpthread"],
)