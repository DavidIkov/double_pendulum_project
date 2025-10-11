cmake_minimum_required(VERSION 3.22)
project(simulation LANGUAGES CXX)

add_library(
  pendulum STATIC
  pendulum/pendulum.hpp pendulum/simulation.hpp pendulum/double.hpp
  pendulum/double.cpp pendulum/single.hpp pendulum/single.cpp)
target_link_libraries(pendulum PUBLIC mathcpp)

add_library(renderer STATIC visualizer/renderer/renderer.hpp
                            visualizer/renderer/renderer.cpp)
target_link_libraries(renderer PUBLIC mathcpp)

add_executable(visualizer_single visualizer/single.cpp)
target_link_libraries(visualizer_single PRIVATE pendulum renderer)
target_include_directories(visualizer_single PRIVATE ${PROJECT_SOURCE_DIR})

add_executable(visualizer_double visualizer/double.cpp)
target_link_libraries(visualizer_double PRIVATE pendulum renderer)
target_include_directories(visualizer_double PRIVATE ${PROJECT_SOURCE_DIR})
