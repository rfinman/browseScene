cmake_minimum_required(VERSION 2.6.0)

find_package(PkgConfig)
pkg_check_modules(GTKMM gtkmm-2.4)

# Automatically build LCM types.
#include(cmake/pods.cmake)
#include(cmake/lcmtypes.cmake)
#lcmtypes_build()

#link_libraries(${LCMTYPES_LIBS})

add_subdirectory(src)
