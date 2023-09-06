cmake_minimum_required(VERSION 3.15)
project(testing)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
    ${CMAKE_MODULE_PATH})

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/mvfst/_build")

find_package(folly CONFIG REQUIRED)
find_package(gflags CONFIG REQUIRED)
find_package(mvfst REQUIRED)

add_executable(${PROJECT_NAME} hello.cpp)

target_link_libraries(${PROJECT_NAME} mvfst::mvfst_client)
