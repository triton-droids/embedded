cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/version.cmake)
project(esp_can_test)

# This should be the name of the project, which is also the name of the
# hoofdirectory of the project.
include($ENV{IDF_PATH}/tools/cmake/idf.cmake)

project_name(esp_can_test)

idf_build_executable(
    "${CMAKE_PROJECT_NAME}.elf"
    SOURCES "main/main.c"
)
