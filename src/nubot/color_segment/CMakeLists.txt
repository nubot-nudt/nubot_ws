cmake_minimum_required(VERSION 2.8.3)
project(color_segment)
#set(EXECUTABLE_OUTPUT_PATH  "${PROJECT_SOURCE_DIR}/bin")
set(CMAKE_PREFIX_PATH  ${CMAKE_PREFIX_PATH} "/home/winston/Qt5.3.2/5.3/gcc_64/lib/cmake/Qt5Widgets/")
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
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
 
find_package(Qt5Widgets)  
find_package(OpenCV REQUIRED)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES color_segment
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport message_filters message_generation roscpp rospy sensor_msgs std_msgs std_srvs
#  DEPENDS system_lib
)

set(color_segment_includes
               ${catkin_INCLUDE_DIRS}
               ${PROJECT_SOURCE_DIR}/include/color_segment
)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${color_segment_includes})

FILE(GLOB_RECURSE CPP_SRC_LIST "src/*.cpp")

Qt5_WRAP_CPP(calibration_MOCS include/color_segment/calibration_dialog.h)
Qt5_WRAP_UI(calibration_UIS src/calibration_dialog.ui)
add_executable(color_segment_node
     ${CPP_SRC_LIST}
     ${calibration_UIS}
     ${calibration_MOCS}
)

target_link_libraries(color_segment_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
qt5_use_modules(color_segment_node Widgets)
 
