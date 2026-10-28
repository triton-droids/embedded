# Ubuntu 22.04's libzstd-dev ships no CMake config, but encos_driver uses
# find_package(zstd CONFIG). Only needed while building encos_driver: it links
# zstd privately, so its installed package does not look for zstd.
find_library(ZSTD_LIBRARY NAMES zstd REQUIRED)
find_path(ZSTD_INCLUDE_DIR zstd.h REQUIRED)
if(NOT TARGET zstd::libzstd_shared)
  add_library(zstd::libzstd_shared SHARED IMPORTED)
  set_target_properties(zstd::libzstd_shared PROPERTIES
    IMPORTED_LOCATION "${ZSTD_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${ZSTD_INCLUDE_DIR}")
endif()
