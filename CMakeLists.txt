cmake_minimum_required(VERSION 3.15)
project(testing)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
    ${CMAKE_MODULE_PATH})

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/libs/mvfst/_build")

find_package(fizz CONFIG REQUIRED)
find_package(folly CONFIG REQUIRED)
find_package(gflags CONFIG REQUIRED)
find_package(gtest CONFIG REQUIRED)
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
