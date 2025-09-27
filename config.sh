#! /bin/bash

cmake -S. -Bbuild \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_C_COMPILER="clang" -DCMAKE_CXX_COMPILER="clang++"\
    -DCMAKE_GENERATOR="Ninja"\
    -DCMAKE_BUILD_TYPE="Debug"
