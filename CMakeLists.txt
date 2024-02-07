cmake_minimum_required(VERSION 3.15)
project(testing)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CC_STANDARD 17)

set(CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
    ${CMAKE_MODULE_PATH})

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/libs/mvfst/_build")
list(APPEND CMAKE_PREFIX_PATH "/usr/local")

find_package(PkgConfig REQUIRED)

## Octree
pkg_search_module(WEBP REQUIRED libwebp)
pkg_search_module(JPEG REQUIRED libjpeg)
pkg_search_module(AVIF REQUIRED libavif)
find_package(Eigen3 3.1.2)
find_package(PCL 1.1.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(octree src/main.cpp src/common.cpp)

target_link_libraries(
    octree
    ${WEBP_LIBRARIES}
    ${JPEG_LIBRARIES}
    ${PCL_LIBRARIES}
    ${AVIF_LIBRARIES}
)

## mvfst 
find_package(fizz CONFIG REQUIRED)
find_package(folly CONFIG REQUIRED)
find_package(gflags CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)
find_package(mvfst REQUIRED)

add_executable(quic_test quic_test/main.cpp)

target_link_libraries(
    quic_test 
    fizz::fizz
    fizz::fizz_test_support
    GTest::gtest
    GTest::gtest_main
    mvfst::mvfst_constants
    mvfst::mvfst_exception
    mvfst::mvfst_transport
    mvfst::mvfst_client
    mvfst::mvfst_socketutil
    mvfst::mvfst_transport_knobs
    mvfst::mvfst_fizz_client
    mvfst::mvfst_fizz_handshake
    mvfst::mvfst_server
    mvfst::mvfst_state_datagram_handler
)

## Quiche (Cloudflare)

add_custom_target(
    quiche_rust
    COMMAND cd ./../libs/quiche  && cargo build --target-dir ./../../third_party_build --features ffi
)

add_executable(quiche_transport src/quiche_transport/client.cc src/quiche_transport/server.cc)
add_executable(hello_quiche src/quiche_transport/hello.cc)

add_dependencies(
    quiche_transport
    quiche_rust
)

find_library(LIBQUICHE
    NAMES quiche
    HINTS ./third_party_build/debug
)

target_include_directories(quiche_transport PUBLIC ./libs/quiche/quiche/include)

target_link_libraries(
    quiche_transport
    ssl
    crypto
    ev
    dl
    m
    pthread
    ${LIBQUICHE}
)