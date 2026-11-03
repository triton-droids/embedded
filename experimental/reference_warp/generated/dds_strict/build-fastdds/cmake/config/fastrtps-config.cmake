# Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(fastrtps_VERSION 2.14.5)


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

if(MSVC OR MSVC_IDE)
    set_and_check(fastrtps_BIN_DIR "${PACKAGE_PREFIX_DIR}/bin")
endif()
set_and_check(fastrtps_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/include")
set_and_check(fastrtps_LIB_DIR "${PACKAGE_PREFIX_DIR}/lib")

find_package(fastcdr REQUIRED)
find_package(foonathan_memory REQUIRED)
find_package(TinyXML2 QUIET)

find_package(OpenSSL REQUIRED)

set(fastrtps_known_comps static shared)
set(fastrtps_comp_static NO)
set(fastrtps_comp_shared NO)
foreach (fastrtps_comp IN LISTS ${CMAKE_FIND_PACKAGE_NAME}_FIND_COMPONENTS)
    if (fastrtps_comp IN_LIST fastrtps_known_comps)
        set(fastrtps_comp_${fastrtps_comp} YES)
    else ()
        set(${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE
            "fastrtps does not recognize component `${fastrtps_comp}`.")
        set(${CMAKE_FIND_PACKAGE_NAME}_FOUND FALSE)
        return()
    endif ()
endforeach ()

if (fastrtps_comp_static AND fastrtps_comp_shared)
    set(${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE
        "fastrtps `static` and `shared` components are mutually exclusive.")
    set(${CMAKE_FIND_PACKAGE_NAME}_FOUND FALSE)
    return()
endif ()

set(fastrtps_static_targets "${CMAKE_CURRENT_LIST_DIR}/fastrtps-static-targets.cmake")
set(fastrtps_shared_targets "${CMAKE_CURRENT_LIST_DIR}/fastrtps-shared-targets.cmake")

macro(fastrtps_load_targets type)
    if (NOT EXISTS "${fastrtps_${type}_targets}")
        set(${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE
            "fastrtps `${type}` libraries were requested but not found.")
        set(${CMAKE_FIND_PACKAGE_NAME}_FOUND FALSE)
        return()
    endif ()

    # find related dependency packages like OpenSSL
    
    

    include("${fastrtps_${type}_targets}")
endmacro()

if (fastrtps_comp_static)
    fastrtps_load_targets(static)
elseif (fastrtps_comp_shared)
    fastrtps_load_targets(shared)
elseif (DEFINED fastrtps_SHARED_LIBS AND fastrtps_SHARED_LIBS)
    fastrtps_load_targets(shared)
elseif (DEFINED fastrtps_SHARED_LIBS AND NOT fastrtps_SHARED_LIBS)
    fastrtps_load_targets(static)
elseif (BUILD_SHARED_LIBS)
    if (EXISTS "${fastrtps_shared_targets}")
        fastrtps_load_targets(shared)
    else ()
        fastrtps_load_targets(static)
    endif ()
else ()
    if (EXISTS "${fastrtps_static_targets}")
        fastrtps_load_targets(static)
    else ()
        fastrtps_load_targets(shared)
    endif ()
endif ()

include(${CMAKE_CURRENT_LIST_DIR}/optionparser-targets.cmake)
