cmake_minimum_required(VERSION 2.4.6)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing ")
endif()

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

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

cmake_minimum_required (VERSION 2.4)
if(POLICY CMP0015)
   cmake_policy(SET CMP0015 NEW)
endif()

message(STATUS "CMAKE_SOURCE_DIR : " ${CMAKE_SOURCE_DIR})
message(STATUS "CMAKE_MODULE_PATH : " ${CMAKE_MODULE_PATH})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../cmake")

if( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )
  set(PLUGIN_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../plugins")
else( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )
    set(PLUGIN_INSTALL_DIR "$ENV{PLUGIN_INSTALL_DIR}")
endif( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )

message(STATUS "PLUGIN_INSTALL_DIR : " ${PLUGIN_INSTALL_DIR})

# Find important packages
# --------------------------------------------------
find_package(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})
#include_directories("/home/jmainpri/fuerte_workspace/openni_tracker/msg_gen/cpp/include")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "EIGEN3_INCLUDE_DIR : " ${EIGEN3_INCLUDE_DIR})

find_package(OpenRAVE REQUIRED)
include_directories(${OPENRAVE_INCLUDE_DIRS})
message(STATUS "OPENRAVE_INCLUDE_DIRS : " ${OPENRAVE_INCLUDE_DIRS})
message(STATUS "OPENRAVE_LIBRARY : " ${OPENRAVE_LIBRARY})

if( NOT OPENRAVE_FOUND )
    message(FATAL_ERROR "could not find openrave, check for openrave-config in path")
endif( NOT OPENRAVE_FOUND )

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

link_directories(${OPENRAVE_LINK_DIRS})

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/kinect-main.cpp src/skeletonListener.cpp src/skeletonDrawing.cpp src/recordMotion.cpp src/playMotion.cpp src/classifyMotion.cpp )
target_link_libraries(${PROJECT_NAME} ${OPENRAVE_LIBRARY})
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES
                        LINK_FLAGS ${OPENRAVE_LINK_FLAGS}
                        COMPILE_FLAGS ${OPENRAVE_CXXFLAGS})

install(TARGETS ${PROJECT_NAME} DESTINATION ${PLUGIN_INSTALL_DIR} )

