cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

SET(
  CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake
)
include(cmake/FindMAVCONN.cmake)
include(cmake/FindLCM.cmake)
include(cmake/FindMAVLINK.cmake)
include(cmake/FindGLIB2.cmake)


FIND_PACKAGE(MAVCONN)
FIND_PACKAGE(LCM)
FIND_PACKAGE(MAVLINK  REQUIRED)
FIND_PACKAGE(GLIB2  REQUIRED)
FIND_PACKAGE(GTHREAD2  REQUIRED)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

INCLUDE_DIRECTORIES(
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
  ${MAVCONN_INCLUDE_DIR}
  ${MAVLINK_INCLUDE_DIR}
  ./include/
)

#rosbuild_add_executable(lcmtoros src/lcmtoros.cpp)
#target_link_libraries(lcmtoros glib-2.0 lcm mavconn_lcm)

#rosbuild_add_executable(rostolcm src/rostolcm.cpp)
#target_link_libraries(rostolcm lcm mavconn_lcm)

rosbuild_add_executable(ros_receiver_example src/ros_receiver_example)
target_link_libraries(ros_receiver_example
  ${GLIB2_LIBRARY})

rosbuild_add_executable(mavlink_bridge_serial src/mavlink_ros_serial.cpp)
target_link_libraries(mavlink_bridge_serial 
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY})
