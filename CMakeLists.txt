cmake_minimum_required(VERSION 3.22)
project(project LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(third_party/mathcpp)
add_subdirectory(third_party/pybind11)
add_subdirectory(simulation)
add_subdirectory(balancing)
