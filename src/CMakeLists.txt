cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
#set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" $CMAKE_MODULE_PATH})

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Pangolin REQUIRED)
find_package(CVD REQUIRED)

link_directories( ${GTKMM_LIBRARY_DIRS})
include_directories( ${GTKMM_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(
            ${Pangolin_INCLUDE_DIR}
            ${CG_INCLUDE_PATH}
            ${EIGEN3_INCLUDE_DIR}
            ${CVD_INCLUDE_DIR}
            ${OPENGL_INCLUDE_DIR}
            )

LINK_LIBRARIES(
            ${Pangolin_LIBRARY}
            ${Eigen3_LIBRARY}
            ${CVD_LIBRARY}
            -lGLEW
            -lglut
            -lGLU
            -lGL
            )


SET(TINYOBJLOADER_SOURCES
            ./utils/map_object_label2training_label.h
            ./tinyobjloader/tiny_obj_loader.h
            ./tinyobjloader/tiny_obj_loader.cc
            ./tinyobjloader/obj_writer.h
            ./tinyobjloader/obj_writer.cc
            )

ADD_EXECUTABLE(browse ${TINYOBJLOADER_SOURCES} 
                      browse.cpp 
                      joystick/JoystickController.cpp 
                      joystick/joystick.cpp
                      )

target_link_libraries(browse ${GTKMM_LIBRARIES})

ADD_EXECUTABLE(render_annotated_image ${TINYOBJLOADER_SOURCES} 
		      main_annotation_render.cpp)

target_link_libraries(render_annotated_image ${GTKMM_LIBRARIES})
