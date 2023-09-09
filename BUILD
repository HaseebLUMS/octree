load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_pcl//bzl:pcl.bzl", "pcl_config")
pcl_config()

cc_library(
    name = "macwebp",
    srcs = [
        "libs/libwebp-mac/lib/libwebp.a",
        "libs/libwebp-mac/include/webp/encode.h",
        "libs/libwebp-mac/include/webp/decode.h",
        "libs/libwebp-mac/include/webp/types.h"
    ],
    visibility = ["//main:__pkg__"],
)

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
    deps = ["//:macwebp"],
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
    srcs = ["src/main.cc", "src/encode.hpp", "src/decode.hpp", "src/common.h", "src/common.cc"],
    deps = ["@pcl//:common", "@pcl//:octree", "@pcl//:io", "//:macjpegencdec", "//:macwebp"],
    data = ["assets", "output"],
)

cc_binary(
    name = "server",
    srcs = ["src/transport/server.cc", "src/transport/config.h"],
    linkopts = ["-lpthread"],
    data = ["assets", "output"]
)

cc_binary(
    name = "aclient",
    srcs = ["src/transport/auto_client.cc", "src/transport/client.hpp", "src/transport/config.h"],
    linkopts = ["-lpthread"],
    data = ["assets", "output"]
)

cc_binary(
    name = "mclient",
    srcs = ["transport/man_client.cc", "transport/client.hpp", "transport/config.h"],
    linkopts = ["-lpthread"],
    data = ["assets", "output"]
)