cmake_minimum_required(VERSION 2.8.3)
project(omni_vision)
#  std=gnu++0x" enables c++ 11
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

find_package(OpenMP REQUIRED)

set(CMAKE_BUILD_TYPE "Release")
if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread -fopenmp")
else( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread -fopenmp")
endif( CMAKE_BUILD_TYPE STREQUAL "Debug" )

find_package(catkin REQUIRED COMPONENTS
  nubot_common
  cv_bridge
  dynamic_reconfigure
  image_transport
  message_filters
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)
find_package(OpenCV REQUIRED)


generate_dynamic_reconfigure_options(cfg/OmniVision.cfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport message_filters message_generation opencv2/ roscpp rospy sensor_msgs std_msgs std_srvs
)

set(omni_vision_includes
               ${catkin_INCLUDE_DIRS}
               ${PROJECT_SOURCE_DIR}/include
               ${Boost_INCLUDE_DIRS}
               ${Eigen_INCLUDE_DIRS}
)
include_directories(${omni_vision_includes})
add_executable(omni_vision_node
      src/omnivision.cpp
      src/localization.cpp  
      src/omniimage.cpp         
      src/scanpoints.cpp
      src/fieldinformation.cpp
      src/optimise.cpp          
      src/transfer.cpp
      src/glocalization.cpp     
      src/odometry.cpp       
      src/whites.cpp
      src/colorsegment.cpp
      src/ballfinder.cpp
      src/obstacles.cpp
)
add_dependencies(omni_vision_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(omni_vision_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )

