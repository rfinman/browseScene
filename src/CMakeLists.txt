cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" $CMAKE_MODULE_PATH})
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})

SET(TINYOBJLOADER_SOURCES
./utils/map_object_label2training_label.h
./tinyobjloader/tiny_obj_loader.h
./tinyobjloader/tiny_obj_loader.cc
./tinyobjloader/obj_writer.h
./tinyobjloader/obj_writer.cc
)

##############################################
add_executable (browse browse.cpp)
target_link_libraries (browse ${TINYOBJLOADER_SOURCES} ${OPENGL_LIBRARIES} 
${GLUT_LIBRARIES})

