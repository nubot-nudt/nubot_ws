cmake_minimum_required(VERSION 2.8.3)

project(prosilica_camera)

set(CMAKE_BUILD_TYPE "Release")
if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread")
else( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread -fopenmp")
endif( CMAKE_BUILD_TYPE STREQUAL "Debug" )

find_package(Boost  REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
  prosilica_gige_sdk
  roscpp 
  sensor_msgs 
  std_msgs
  driver_base
  message_generation
  diagnostic_updater
  polled_camera
  self_test
  diagnostic_msgs
  image_transport 
  camera_calibration_parsers
  dynamic_reconfigure
)

find_package(PkgConfig)
generate_dynamic_reconfigure_options(cfg/ProsilicaCamera.cfg)

catkin_package(CATKIN_DEPENDS std_msgs roscpp dynamic_reconfigure)
set(prosilica_camera_include
         include
         ${prosilica_INCLUDE_DIRS}
         ${catkin_INCLUDE_DIRS}
)

include_directories(${prosilica_camera_include})
add_subdirectory(src)


