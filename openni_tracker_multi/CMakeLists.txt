cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_make_distribution(0.1.3)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include(FindPkgConfig)
pkg_check_modules(OPENNI libopenni)


include_directories(${OPENNI_INCLUDEDIR} /usr/include/nite)

rosbuild_add_executable(tracker1 src/openni_tracker1.cpp)
target_link_libraries(tracker1 ${OPENNI_LIBRARIES})

rosbuild_add_executable(tracker2 src/openni_tracker2.cpp)
target_link_libraries(tracker2 ${OPENNI_LIBRARIES})
