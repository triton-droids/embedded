#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "CycloneDDS::idlc" for configuration "None"
set_property(TARGET CycloneDDS::idlc APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(CycloneDDS::idlc PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/bin/idlc"
  )

list(APPEND _cmake_import_check_targets CycloneDDS::idlc )
list(APPEND _cmake_import_check_files_for_CycloneDDS::idlc "${_IMPORT_PREFIX}/bin/idlc" )

# Import target "CycloneDDS::idl" for configuration "None"
set_property(TARGET CycloneDDS::idl APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(CycloneDDS::idl PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libcycloneddsidl.so.0.10.5"
  IMPORTED_SONAME_NONE "libcycloneddsidl.so.0"
  )

list(APPEND _cmake_import_check_targets CycloneDDS::idl )
list(APPEND _cmake_import_check_files_for_CycloneDDS::idl "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libcycloneddsidl.so.0.10.5" )

# Import target "CycloneDDS::dds_security_ac" for configuration "None"
set_property(TARGET CycloneDDS::dds_security_ac APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(CycloneDDS::dds_security_ac PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libdds_security_ac.so"
  IMPORTED_SONAME_NONE "libdds_security_ac.so"
  )

list(APPEND _cmake_import_check_targets CycloneDDS::dds_security_ac )
list(APPEND _cmake_import_check_files_for_CycloneDDS::dds_security_ac "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libdds_security_ac.so" )

# Import target "CycloneDDS::dds_security_auth" for configuration "None"
set_property(TARGET CycloneDDS::dds_security_auth APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(CycloneDDS::dds_security_auth PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libdds_security_auth.so"
  IMPORTED_SONAME_NONE "libdds_security_auth.so"
  )

list(APPEND _cmake_import_check_targets CycloneDDS::dds_security_auth )
list(APPEND _cmake_import_check_files_for_CycloneDDS::dds_security_auth "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libdds_security_auth.so" )

# Import target "CycloneDDS::dds_security_crypto" for configuration "None"
set_property(TARGET CycloneDDS::dds_security_crypto APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(CycloneDDS::dds_security_crypto PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libdds_security_crypto.so"
  IMPORTED_SONAME_NONE "libdds_security_crypto.so"
  )

list(APPEND _cmake_import_check_targets CycloneDDS::dds_security_crypto )
list(APPEND _cmake_import_check_files_for_CycloneDDS::dds_security_crypto "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libdds_security_crypto.so" )

# Import target "CycloneDDS::ddsc" for configuration "None"
set_property(TARGET CycloneDDS::ddsc APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(CycloneDDS::ddsc PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NONE "iceoryx_binding_c::iceoryx_binding_c"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libddsc.so.0.10.5"
  IMPORTED_SONAME_NONE "libddsc.so.0"
  )

list(APPEND _cmake_import_check_targets CycloneDDS::ddsc )
list(APPEND _cmake_import_check_files_for_CycloneDDS::ddsc "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libddsc.so.0.10.5" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
