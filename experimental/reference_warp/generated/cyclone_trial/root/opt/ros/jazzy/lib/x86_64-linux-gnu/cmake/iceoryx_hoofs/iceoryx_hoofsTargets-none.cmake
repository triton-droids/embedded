#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "iceoryx_hoofs::iceoryx_hoofs" for configuration "None"
set_property(TARGET iceoryx_hoofs::iceoryx_hoofs APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(iceoryx_hoofs::iceoryx_hoofs PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libiceoryx_hoofs.so"
  IMPORTED_SONAME_NONE "libiceoryx_hoofs.so"
  )

list(APPEND _cmake_import_check_targets iceoryx_hoofs::iceoryx_hoofs )
list(APPEND _cmake_import_check_files_for_iceoryx_hoofs::iceoryx_hoofs "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libiceoryx_hoofs.so" )

# Import target "iceoryx_hoofs::iceoryx_platform" for configuration "None"
set_property(TARGET iceoryx_hoofs::iceoryx_platform APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(iceoryx_hoofs::iceoryx_platform PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libiceoryx_platform.so"
  IMPORTED_SONAME_NONE "libiceoryx_platform.so"
  )

list(APPEND _cmake_import_check_targets iceoryx_hoofs::iceoryx_platform )
list(APPEND _cmake_import_check_files_for_iceoryx_hoofs::iceoryx_platform "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libiceoryx_platform.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
