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
include(FetchContent)

# ## ReaderWriterQueue
# FetchContent_Declare(
#   readerwriterqueue
#   GIT_REPOSITORY    https://github.com/cameron314/readerwriterqueue
#   GIT_TAG           master
# )

# FetchContent_MakeAvailable(readerwriterqueue)

# Octree
pkg_search_module(WEBP REQUIRED libwebp)
pkg_search_module(JPEG REQUIRED libjpeg)
# pkg_search_module(AVIF REQUIRED libavif)
# pkg_search_module(AVIF REQUIRED libavif)
pkg_search_module(TURBOJPEG REQUIRED libturbojpeg)

find_package(Eigen3 3.1.2)
find_package(PCL 1.1.0 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(jsoncpp REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${OpenCV_INCLUDE_DIRS})


link_directories(${TURBOJPEG_LIBDIR})

add_executable(octree src/main.cpp src/common.cpp)
add_executable(ssim src/quality_test/ssim.cpp)

target_link_libraries(
    octree
    ${WEBP_LIBRARIES}
    ${JPEG_LIBRARIES}
    ${PCL_LIBRARIES}
    ${AVIF_LIBRARIES}
    ${TURBOJPEG_LIBRARIES}
    # readerwriterqueue
)

target_link_libraries(
    ssim
    ${WEBP_LIBRARIES}
    ${JPEG_LIBRARIES}
    ${PCL_LIBRARIES}
    ${AVIF_LIBRARIES}
    ${TURBOJPEG_LIBRARIES}
    ${OpenCV_LIBS}
    jsoncpp_lib
    # readerwriterqueue
)

## Unix Transport

add_executable(server src/transport/server.cc)
add_executable(client src/transport/man_client.cc)
add_executable(auto_client src/transport/auto_client.cc)

target_link_libraries(
    server
    pthread

)

target_link_libraries(
    client
    pthread
)

target_link_libraries(
    auto_client
    pthread
)

## Quiche (Cloudflare)

add_custom_target(
    quiche_rust
    COMMAND cd ./../libs/quiche  && cargo build --target-dir ./../../third_party_build --features ffi
)

add_executable(quiche_server src/quiche_transport/server.cc)
add_executable(quiche_client src/quiche_transport/client.cc)
add_executable(hello_quiche src/quiche_transport/hello.cc)

add_dependencies(
    quiche_server
    quiche_rust
)

add_dependencies(
    quiche_client
    quiche_rust
)

find_library(LIBQUICHE
    NAMES quiche
    HINTS ./third_party_build/debug
)

target_include_directories(quiche_server PUBLIC ./libs/quiche/quiche/include)
target_include_directories(quiche_client PUBLIC ./libs/quiche/quiche/include)

target_link_libraries(
    quiche_server
    ssl
    crypto
    ev
    dl
    m
    pthread
    ${LIBQUICHE}
)

target_link_libraries(
    quiche_client
    ssl
    crypto
    ev
    dl
    m
    pthread
    ${LIBQUICHE}
)