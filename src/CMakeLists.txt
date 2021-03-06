cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
#set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" $CMAKE_MODULE_PATH})

FIND_PACKAGE(CUDA REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Pangolin REQUIRED)
find_package(CVD REQUIRED)
find_package(MinimalImgUtilities REQUIRED)
find_package(OpenMP)
find_package(Boost COMPONENTS thread REQUIRED)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


# Adapt it accordingly
SET(CUDA_NVCC_FLAGS  "-arch=sm_30"  "--use_fast_math" "-O3" "-lineinfo")

FIND_PATH(CUDA_CUT_INCLUDE_DIR
   cutil.h
   PATHS ${CUDA_SDK_SEARCH_PATH}
   PATH_SUFFIXES "common/inc"
   DOC "Location of cutil.h"
   NO_DEFAULT_PATH
)


message("CUDA_SDK_SEARCH_PATH = ${CUDA_SDK_SEARCH_PATH}")

link_directories( ${GTKMM_LIBRARY_DIRS})
include_directories( ${GTKMM_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(
            ${Pangolin_INCLUDE_DIR}
            ${CG_INCLUDE_PATH}
            ${EIGEN3_INCLUDE_DIR}
            ${CVD_INCLUDE_DIR}
            ${OPENGL_INCLUDE_DIR}
	    ${MinimalImgUtilities_INCLUDE_DIR}
            ${CUDA_CUT_INCLUDE_DIR}
            ${CUDA_INCLUDE_DIRS}
	    /usr/local/include/sigc++-2.0/
	    /usr/local/lib/sigc++-2.0/include/
	    /usr/lib/x86_64-linux-gnu/glibmm-2.4/include/
             /usr/include/glib-2.0/
/usr/lib/x86_64-linux-gnu/glib-2.0/include/
/usr/include/glibmm-2.4
            )

LINK_LIBRARIES(
            ${Pangolin_LIBRARY}
            ${Eigen3_LIBRARY}
            ${CVD_LIBRARY}
	    ${MinimalImgUtilities_LIBRARY}
            -lGLEW
            -lglut
            -lGLU
            -lGL
   	    #-lglib
  	/usr/local/lib/libsigc-2.0.so
/usr/lib/x86_64-linux-gnu/libglib-2.0.so
            )

LINK_DIRECTORIES(
  ${OPENCV_LIBRARY_DIRS}
  ${CUDA_SDK_ROOT_DIR}/lib
  ${CUDA_SDK_ROOT_DIR}/common/lib
  /usr/local/lib
  /usr/local/lib64/osgPlugins-3.3.3
  /usr/local/lib64
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
                      convert_poses.cpp
                      )
target_link_libraries(browse ${GTKMM_LIBRARIES} -lboost_system boost_thread)

ADD_EXECUTABLE(render_annotated_image ${TINYOBJLOADER_SOURCES} 
		      main_annotation_render.cpp)
target_link_libraries(render_annotated_image ${GTKMM_LIBRARIES})

ADD_EXECUTABLE(create_script create_povray_script.cpp convert_poses.cpp)

# Add png_to lcm files and lcmtypes
ADD_EXECUTABLE(png_to_lcm utils/png_to_lcm.cpp 
utils/lcmtypes/openni_frame_msg_t.c utils/lcmtypes/openni_depth_msg_t.c 
utils/lcmtypes/openni_image_msg_t.c utils/lcmtypes/openni_disparity_msg_t.c)
target_link_libraries(png_to_lcm -llcm)

SET(KINECTNOISE_SOURCES
    ./noise/add_kinect_noise.h
    ./noise/add_kinect_noise.cu)

SET(VaFRIC_SOURCES
    ./VaFRIC/VaFRIC.h
    ./VaFRIC/VaFRIC.cpp)


#SET(SHADERS_SOURCES
#    ./shaders/glsl_shader.h
#    ./shaders/glsl_shader.cpp)

CUDA_ADD_EXECUTABLE(add_kinect_noise
                    ${VaFRIC_SOURCES}
                    ${KINECTNOISE_SOURCES}
                    main_add_kinect_noise.cpp)

#CUDA_ADD_EXECUTABLE(depth_shader
#                    ${VaFRIC_SOURCES}
#                    ${SHADERS_SOURCES}
#                    main_depth_shader.cpp)
