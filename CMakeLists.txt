
Project(WISCO_OpenSim)
cmake_minimum_required(VERSION 3.3)

set(WISCO_OPENSIM_DIR "C:/github/wisco_opensim/source" CACHE
        PATH "Top-level directory of wisco_opensim repo")

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
      SET(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../install" CACHE PATH "Location to install plugin libraries" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

#Specify OpenSim Stuff
#===============================================================================
set(OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME}
    CACHE PATH "Top-level directory of OpenSim install")

# OpenSim uses C++11 language features.
if("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU" OR
        "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    # Using C++11 on OSX requires using libc++ instead of libstd++.
    # libc++ is an implementation of the C++ standard library for OSX.
    if(APPLE)
        if(XCODE)
            set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
            set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
        else()
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
            if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
            endif()
        endif()
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
endif()

find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

include_directories(${OpenSim_INCLUDE_DIRS})
link_directories(${OpenSim_LIB_DIR})

# Find external Libraries
#===============================================================================
FILE(GLOB EXTERNAL_LIB_NAMES RELATIVE "${WISCO_OPENSIM_DIR}/dependencies" "${WISCO_OPENSIM_DIR}/dependencies/*")
FOREACH(external_lib_name ${EXTERNAL_LIB_NAMES})
  include_directories(${WISCO_OPENSIM_DIR}/dependencies/${external_lib_name}/)
ENDFOREACH(external_lib_name)

include_directories(${WISCO_OPENSIM_DIR}/../install-dependencies/HDF5/include/)

#Add Subdirectories
#===============================================================================
add_subdirectory(plugin)
include_directories("${WISCO_OPENSIM_DIR}/plugin")
add_subdirectory(sandbox)
add_subdirectory(tests)
add_subdirectory(examples)
