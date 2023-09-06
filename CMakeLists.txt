cmake_minimum_required(VERSION 3.15)
project(testing)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
    ${CMAKE_MODULE_PATH})

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/mvfst/_build")

find_package(fizz CONFIG REQUIRED)
find_package(folly CONFIG REQUIRED)
find_package(gflags CONFIG REQUIRED)
find_package(gtest CONFIG REQUIRED)
find_package(mvfst REQUIRED)

add_executable(hello hello.cpp)
add_executable(echo echo/main.cpp)

target_link_libraries(hello mvfst::mvfst_client)
target_link_libraries(
    echo 
    fizz::fizz
    fizz::fizz_test_support
    GTest::gtest
    GTest::gtest_main
    mvfst::mvfst_constants
    mvfst::mvfst_exception
    mvfst::mvfst_transport
    mvfst::mvfst_client
    mvfst::mvfst_codec_types
    mvfst::mvfst_codec_decode
    mvfst::mvfst_codec_pktbuilder
    mvfst::mvfst_codec_pktrebuilder
    mvfst::mvfst_codec_packet_number_cipher
    mvfst::mvfst_codec
    mvfst::mvfst_looper
    mvfst::mvfst_buf_accessor
    mvfst::mvfst_bufutil
    mvfst::mvfst_socketutil
    mvfst::mvfst_transport_knobs
    mvfst::mvfst_cc_algo
    mvfst::mvfst_dsr_types
    mvfst::mvfst_dsr_frontend
    mvfst::mvfst_fizz_client
    mvfst::mvfst_fizz_handshake
    mvfst::mvfst_flowcontrol
    mvfst::mvfst_handshake
    mvfst::mvfst_happyeyeballs
    mvfst::mvfst_qlogger
    mvfst::mvfst_loss
    mvfst::mvfst_observer
    mvfst::mvfst_server
    mvfst::mvfst_server_async_tran
    mvfst::mvfst_server_state
    mvfst::mvfst_state_machine
    mvfst::mvfst_state_ack_handler
    mvfst::mvfst_state_datagram_handler
    mvfst::mvfst_state_stream_functions
    mvfst::mvfst_state_pacing_functions
    mvfst::mvfst_state_functions
    mvfst::mvfst_state_simple_frame_functions
    mvfst::mvfst_state_stream
)
