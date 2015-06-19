cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" $CMAKE_MODULE_PATH})
find_package(PkgConfig REQUIRED)

#GTKMM files have sigc++
#pkg_check_modules(SIGC2 REQUIRED sigc++-2.0)
#include_directories(${SIGC2_INCLUDE_DIRS})

link_directories( ${GTKMM_LIBRARY_DIRS})
include_directories( ${GTKMM_INCLUDE_DIRS})

##############################################
add_executable (browse browse.cpp)
#target_link_libraries (main ${GTKMM_LIBRARIES} ${SIGC2_INCLUDE_DIRS})
target_link_libraries (main ${GTKMM_LIBRARIES} )

