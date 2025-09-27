cmake_minimum_required(VERSION 3.22)
project(simulation LANGUAGES CXX)

add_library(pendulum STATIC pendulum/double.hpp pendulum/double.cpp)
target_link_libraries(pendulum PUBLIC mathcpp)

add_executable(visualizer visualizer/main.cpp)
target_link_libraries(visualizer PRIVATE pendulum)
target_include_directories(visualizer PRIVATE ${PROJECT_SOURCE_DIR})

