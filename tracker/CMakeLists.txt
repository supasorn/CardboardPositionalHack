cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(tracker)
FIND_PACKAGE(OpenCV 2.4 REQUIRED)
SET(EXTRA_LIBS GL glut GLU /usr/local/lib/libgflags.a)
SET(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} -lpthread -pthread")

add_definitions(-fopenmp)
add_definitions(-D__UBUNTU__)
add_definitions(-std=c++0x)
FIND_PACKAGE(CUDA)
SET(CUDA_NVCC_FLAGS "-arch=sm_20")
SET(OSBASE "/home/supasorn")

message(${CMAKE_SYSTEM_VERSION})
message(${CMAKE_CURRENT_SOURCE_DIR})

FIND_PACKAGE(OpenGL REQUIRED )

message("OPENCV: ${OpenCV_VERSION}")
include_directories(${INCLUDE_DIRECTORIES} ${OPENCV_INCLUDE_DIRS} /usr/local/include/opencv) 

add_executable (glcv3 glcv3.cpp)
target_link_libraries (glcv3 ${OpenCV_LIBS} ${EXTRA_LIBS})

