cmake_minimum_required(VERSION 3.15)
project(testing)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
    ${CMAKE_MODULE_PATH})

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/libs/mvfst/_build")
list(APPEND CMAKE_PREFIX_PATH "/usr/local")

find_package(WebP CONFIG REQUIRED)
find_package(libjpeg-turbo CONFIG REQUIRED)
find_package(libavif CONFIG REQUIRED)
find_package(Eigen3 3.1.2)

find_package(PCL 1.1.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(BEFORE "${CMAKE_CURRENT_SOURCE_DIR}/src")

add_executable(octree src/main.cpp)
target_include_directories(octree PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
# target_link_directories(octree PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

target_link_libraries(
    octree
    WebP::webp
    libjpeg-turbo::jpeg
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    avif
)

# find_package(fizz CONFIG REQUIRED)
# find_package(folly CONFIG REQUIRED)
# find_package(gflags CONFIG REQUIRED)
# find_package(GTest CONFIG REQUIRED)
# find_package(mvfst REQUIRED)

# add_executable(quic_test quic_test/main.cpp)

# target_link_libraries(
#     quic_test 
#     fizz::fizz
#     fizz::fizz_test_support
#     GTest::gtest
#     GTest::gtest_main
#     mvfst::mvfst_constants
#     mvfst::mvfst_exception
#     mvfst::mvfst_transport
#     mvfst::mvfst_client
#     mvfst::mvfst_socketutil
#     mvfst::mvfst_transport_knobs
#     mvfst::mvfst_fizz_client
#     mvfst::mvfst_fizz_handshake
#     mvfst::mvfst_server
#     mvfst::mvfst_state_datagram_handler
# )

