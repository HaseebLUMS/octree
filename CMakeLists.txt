cmake_minimum_required(VERSION 3.15)
project(testing)

set(CMAKE_CXX_STANDARD 17)

find_package(mvfst CONFIG REQUIRED)

add_executable(${PROJECT_NAME} hello.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE mvfst::mvfst_client)
