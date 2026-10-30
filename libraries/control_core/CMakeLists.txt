cmake_minimum_required(VERSION 3.16)
project(control_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(control_core SHARED
  src/api.cpp
  src/operation.cpp
)
# The ROS adapter builds the C++ sources directly and does not need pybind11.
# Standalone users can disable the Python module explicitly.
option(CONTROL_CORE_BUILD_PYTHON "Build the standalone Python bindings" OFF)
if(CONTROL_CORE_BUILD_PYTHON)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
  find_package(pybind11 CONFIG REQUIRED)
  pybind11_add_module(control_core_py python/bindings.cpp)
  target_link_libraries(control_core_py PRIVATE control_core)
endif()

target_include_directories(control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
