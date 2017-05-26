project(coldet)

cmake_minimum_required(VERSION 3.2)

# Settings.
# ---------
set(TARGET coldet CACHE TYPE STRING)
file(GLOB SOURCE_FILES *.h *.cpp *.c)
add_library(coldet STATIC ${SOURCE_FILES})
set_target_properties(coldet PROPERTIES DEBUG_POSTFIX "_d")

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set (CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/../install" CACHE PATH "default install path" FORCE )
endif()

install(TARGETS coldet DESTINATION .)
