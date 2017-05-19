#!/bin/bash

# Delete directories to allow a clean build
cmake -E cmake_echo_color --red ">>>>> Delete directories from old build"
cmake -E remove_directory bin
cmake -E remove_directory build

# Create directories
cmake -E cmake_echo_color --green ">>>>> Create directories from current build"
cmake -E make_directory bin
cmake -E make_directory build

# Invoke CMake on project
cmake -E cmake_echo_color --blue ">>>>> Build project"

#cmake -E chdir build cmake -DCMAKE_BUILD_TYPE=Debug
cmake -E chdir build cmake -DCMAKE_BUILD_TYPE=Debug -DLIBIGL_WITH_OPENGL=ON \
-DLIBIGL_WITH_OPENGL_GLFW=ON -DLIBIGL_WITH_VIEWER=ON -DLIBIGL_WITH_NANOGUI=ON ..
#cmake -DCMAKE_BUILD_TYPE=Release ..

# Build project
sh build.sh
