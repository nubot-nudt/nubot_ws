cmake_minimum_required(VERSION 2.8.3)

project(prosilica_gige_sdk)

find_package(catkin REQUIRED)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
   set(PLATFORM x64 )
else(CMAKE_SIZEOF_VOID_P EQUAL 8)
   set(PLATFORM x86 )
endif(CMAKE_SIZEOF_VOID_P EQUAL 8)

link_directories(${PROJECT_SOURCE_DIR}/lib/${PLATFORM})
link_libraries(PvAPI)

add_definitions(-D_LINUX -D_${PLATFORM})

set(EXAMPLES  ${PROJECT_SOURCE_DIR}/avt_gige_sdk/examples)

set(prosilica_include
    include/
    ${catkin_INCLUDE_DIRS}
)
include_directories(${prosilica_include})

find_library(PVAPI NAMES "PvAPI" PATHS "lib/${PLATFORM}")

catkin_package(
    INCLUDE_DIRS ${prosilica_include}
    LIBRARIES ${PVAPI}
)
add_executable(CLIpConfig      ${EXAMPLES}/CLIpConfig/CLIpConfig.cpp)
add_executable(Ping            ${EXAMPLES}/Ping/Ping.cpp)
add_executable(ListCameras     ${EXAMPLES}/ListCameras/ListCameras.cpp)
add_executable(ResetCamera     ${EXAMPLES}/ResetCamera/ResetCamera.cpp)
add_executable(ListAttributes  ${EXAMPLES}/ListAttributes/ListAttributes.cpp)
