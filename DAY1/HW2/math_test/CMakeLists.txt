cmake_minimum_required(VERSION 3.10)
project(math_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_executable(math_test src/math_test.cpp)
target_link_libraries(math_test Eigen3::Eigen)
