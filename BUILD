load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_pcl//bzl:pcl.bzl", "pcl_config")
pcl_config()

cc_library(
    name = "macjpegencdec",
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

cc_library(
    name = "linuxjpegencdec",
    srcs = [
        "libs/JpegDecoder.cpp",
        "libs/JpegDecoder.hpp",
        "libs/JpegEncoder.cpp",
        "libs/JpegEncoder.hpp",
        "libs/jpeg-turbo/2.1.5.1/include/turbojpeg.h",
    ],
    linkopts = ["-l:/usr/lib/x86_64-linux-gnu/libturbojpeg.a"],
    visibility = ["//main:__pkg__"],
)

cc_binary(
    name = "benchmark",
    srcs = ["benchmark.cc"],
    deps = ["@pcl//:common", "@pcl//:octree", "@pcl//:io", "//:linuxjpegencdec"]
)

cc_binary(
    name = "encdec",
    srcs = ["main.cc", "encode.hpp", "decode.hpp", "common.h", "common.cc"],
    deps = ["@pcl//:common", "@pcl//:octree", "@pcl//:io", "//:linuxjpegencdec"],
    data = ["assets", "output"]
)

cc_binary(
    name = "encdecmac",
    srcs = ["main.cc", "encode.hpp", "decode.hpp", "common.h", "common.cc"],
    deps = ["@pcl//:common", "@pcl//:octree", "@pcl//:io", "//:macjpegencdec"],
    data = ["assets", "output"]
)

cc_binary(
    name = "server",
    srcs = ["transport/server.cc", "transport/config.h"],
    linkopts = ["-lpthread"],
    data = ["assets", "output"]
)

cc_binary(
    name = "aclient",
    srcs = ["transport/auto_client.cc", "transport/client.hpp", "transport/config.h"],
    linkopts = ["-lpthread"],
    data = ["assets", "output"]
)

cc_binary(
    name = "mclient",
    srcs = ["transport/man_client.cc", "transport/client.hpp", "transport/config.h"],
    linkopts = ["-lpthread"],
    data = ["assets", "output"]
)