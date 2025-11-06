project(pendulum_python_export LANGUAGES CXX)

pybind11_add_module(pendulum_python_export impl/export.cpp)

set_target_properties(pendulum_python_export PROPERTIES OUTPUT_NAME
                                                        pendulum_simulation)

target_include_directories(pendulum_python_export
                           PUBLIC ${CMAKE_SOURCE_DIR}/simulation/pendulum)
target_link_libraries(pendulum_python_export PUBLIC pendulum)

install(DIRECTORY pendulum_simulation
        DESTINATION ${CMAKE_BINARY_DIR}/pendulum_python_export)
install(
  TARGETS pendulum_python_export
  DESTINATION ${CMAKE_BINARY_DIR}/pendulum_python_export/pendulum_simulation)
install(DIRECTORY packaging/
        DESTINATION ${CMAKE_BINARY_DIR}/pendulum_python_export)
