cmake_minimum_required(VERSION 3.16)
project(my_project)

set(CMAKE_CXX_STANDARD 17)

find_package(mvfst REQUIRED)

add_executable(MyApp hello.cpp)
target_link_libraries(MyApp mvfst)