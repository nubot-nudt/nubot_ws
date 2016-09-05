cmake_minimum_required(VERSION 2.8.3)
project(front_vision)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
set(CMAKE_BUILD_TYPE "Release")
if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread")
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

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport message_filters message_generation opencv2/ roscpp rospy sensor_msgs std_msgs std_srvs
)
set(front_vision_includes
               ${catkin_INCLUDE_DIRS}
               ${PROJECT_SOURCE_DIR}/include
)

include_directories(${front_vision_includes})
add_executable(front_vision_node
      src/main.cpp
      src/colorsegment.cpp
      src/ballfinder.cpp
)
target_link_libraries(front_vision_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )
add_dependencies(front_vision_node ${catkin_EXPORTED_TARGETS})
