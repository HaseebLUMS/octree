load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_pcl//bzl:pcl.bzl", "pcl_config")
pcl_config()

cc_library(
    name = "mvfst",
    srcs = glob([
        "libs/mvfst/_build/**/lib/*.a",
        "libs/mvfst/_build/**/lib/*.dylib"
    ]),
    hdrs = glob([
        "libs/mvfst/_build/**/include/**/*.h",
        "libs/mvfst/_build/**/include/**/*.hpp"
    ]),
    includes = [
        "libs/mvfst/_build/libevent-8XghsOQRh5Rl2-V4eFu-N1X8ltAAYurshhdZ2XdoNxI/include",
        "libs/mvfst/_build/cmake-P1QC37Ysv-5J65PwwiStTSite2s1srRwGkTq31HmvWc/include",
        "libs/mvfst/_build/libtool-PxEUgV9OsTgFbmYMVXMLQhVVXQ1i22mxNxelq4Kd-3k/include",
        "libs/mvfst/_build/boost-Dtw1s_iPEivg6q0M32NYkYcZINP95NxSwuHnjQPQtO0/include",
        "libs/mvfst/_build/double-conversion-G2inL_Y9ITo-4EQxpomD7rKkt75sdqr-5lsGAPXuNoU/include",
        "libs/mvfst/_build/lz4-p7OoE7Sm1K2LGjJQD8C7DxF1qdjT2y6ty7fM12dIEK0/include",
        "libs/mvfst/_build/ninja-kFbokuZ-QChFpn7osI76jQDwCDjellpNOmgirR-mrt0/include",
        "libs/mvfst/_build/fmt-1n_KwPG76uQqPjSMZpgStKLqVAdr93KgSeWleAjPYCg/include",
        "libs/mvfst/_build/zlib-pSV5ynwzfc5kjiHuxevNIeSuJIry-n1XCzWuGJ25wDo/include",
        "libs/mvfst/_build/snappy-4Jaz2vn7vGio6WyghWI7ZiR40PSLaq8OqhxYiluvizg/include",
        "libs/mvfst/_build/zstd-qoaWKI2a7AFnW3390a9_UJs3vbhukvjqkNyAelSgT_k/include",
        "libs/mvfst/_build/automake-WtyMxSMs6g_xNarv7AN9DUIx0jwP5A8y2OMTKaZRhPg/include",
        "libs/mvfst/_build/libsodium-67aNK5kYYzO3t43uKHoQsSXMA9HyjKYFYi-bRw5Zckc/include",
        "libs/mvfst/_build/zlib-mUL878y_rB_oh4DUwyMQ5UvqJe2-CYRGmGEt8Y6UAg4/include",
        "libs/mvfst/_build/openssl-I4d8NgBIAtuQ1gvqG07YAMtgnvB9zguzLzavZ1kjFdI/include",
        "libs/mvfst/_build/boost-Z5CRLdEuAKQ7y2iRHYz0Vdh33HMWbP2jb4dP_lJIeKs/include",
        "libs/mvfst/_build/xz-2oJqcYH4nsnj8y5lqL4Bj8YZTRQ2uqxA2MTZJeeWwok/include",
        "libs/mvfst/_build/ninja-n2yW1uxkF2fpD4HGZeSo5lBXbRrE9_hMmXl0WpUlWBk/include",
        "libs/mvfst/_build/gflags-ygi5BlmCGHIusXr5YwR9gf2g_rIb4yaFDD8xatPNu8g/include",
        "libs/mvfst/_build/autoconf-qbhp1glvFuK8tD7l880Eg7TA-jIMw6rQAI_KDjlP6FQ/include",
        "libs/mvfst/_build/glog-9SNzWT-dLlysSQoRrhUM-DN6f79eVhXGeJqgi8nJnvg/include",
        "libs/mvfst/_build/mvfst/include",
        "libs/mvfst/_build/cmake-vSWgHu-LwmiXveabjqRx_cn5iUWD5B4IE2IRcs52zsA/include",
        "libs/mvfst/_build/googletest-d1Yzyk2jT3VC0JGUk9VQOe9UFjKP61c2oQnzEocvt8k/include",
        "libs/mvfst/_build/folly/include",
        "libs/mvfst/_build/zstd-R2zFnCNrJ21uB8qPUJAF7RfaE00txdSJenqQGPD3oSk/include",
        "libs/mvfst/_build/fizz/include"
    ],
    linkopts = ["-lc++abi"],
    visibility = ["//main:__pkg__"],
)

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

cc_library(
    name = "macavif",
    srcs = [
        "libs/libavif/1.0.1/include/avif/avif.h",
        "libs/libavif/1.0.1/lib/libavif.dylib"
    ],
    visibility = ["//main:__pkg__"],
)

cc_binary(
    name = "benchmark",
    srcs = ["benchmark.cc"],
    deps = ["@pcl//:common", "@pcl//:octree", "@pcl//:io", "//:linuxjpegencdec"]
)

cc_binary(
    name = "encdec",
    srcs = ["src/main.cc", "src/encode.hpp", "src/decode.hpp", "src/common.h", "src/common.cc", "src/colors.hpp"],
    deps = ["@pcl//:common", "@pcl//:octree", "@pcl//:io", "//:linuxjpegencdec"],
    data = ["assets", "output"]
)

cc_binary(
    name = "encdecmac",
    srcs = ["src/main.cc", "src/encode.hpp", "src/decode.hpp", "src/common.h", "src/common.cc", "src/colors.hpp"],
    deps = ["@pcl//:common", "@pcl//:octree", "@pcl//:io", "//:macjpegencdec", "//:macwebp", "//:macavif"],
    data = ["assets", "output"]
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

cc_binary(
    name = "hello",
    srcs = ["src/hello.cpp"],
    deps = ["//:macavif", "//:mvfst"]
)

cc_binary(
    name = "echo",
    srcs = glob([
        "src/echo/*.h",
        "src/echo/*.cpp"
    ]),
    deps = ["//:mvfst"]
)