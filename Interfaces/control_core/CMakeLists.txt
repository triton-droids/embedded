cmake_minimum_required(VERSION 3.16)
project(control_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(control_core SHARED
  src/api.cpp
)
# ---- Python bindings (pybind11) ----
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)

# Prefer system pybind11 (apt: pybind11-dev). If not found, install via pip and provide CMAKE_PREFIX_PATH.
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(control_core_py
  python/bindings.cpp
)

target_link_libraries(control_core_py PRIVATE control_core)
target_include_directories(control_core_py PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(control_core_demo examples/demo_main.cpp)
target_link_libraries(control_core_demo PRIVATE control_core)
