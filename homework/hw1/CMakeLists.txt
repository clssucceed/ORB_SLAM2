cmake_minimum_required(VERSION 2.8)
project(vo_simulate)

set(CMAKE_BUILD_TYPE release)
set(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
MESSAGE("PROJECT_SOURCE_DIR " ${PROJECT_SOURCE_DIR})

#find_package(OpenCV 3.1)
#if(NOT OpenCV_FOUND)
#   find_package(OpenCV 2.4.13)
#   if(NOT OpenCV_FOUND)
#      message(FATAL_ERROR "OpenCV > 2.4.9 not found.")
#   endif()
#endif()

message("OpenCV_DIR: ${OpenCV_DIR}")
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBS: ${OpenCV_LIBS}")
set(OpenCV_DIR /usr/local/opencv_test/lib/cmake/opencv4)
message("OpenCV_DIR: ${OpenCV_DIR}")
find_package(OpenCV REQUIRED)
message("OpenCV_DIR: ${OpenCV_DIR}")
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBS: ${OpenCV_LIBS}")
find_package(Eigen3  REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OPENCV_INCLUDE_DIR})

add_executable(vo_simulate  demo/main.cpp src/two_view_geometry.cpp)
target_link_libraries(vo_simulate ${OpenCV_LIBS})