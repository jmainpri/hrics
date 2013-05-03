cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_make_distribution(0.1.3)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include(FindPkgConfig)
pkg_check_modules(OPENNI libopenni)

message("OPENNI_INCLUDEDIR : ${OPENNI_INCLUDEDIR}" )
message("OPENNI_LIBRARIES : ${OPENNI_LIBRARIES}" )

set(OPENNI_INCLUDEDIR /home/jmainpri/Downloads/openni/OpenNI-2.1.0/Include )
set(OPENNI_LIBRARIES OpenNI2 )

set(NITE2_INCLUDEDIR /home/jmainpri/Downloads/openni/NiTE-2.0.0/Include )
set(NITE2_LIBRARIES NiTE2 )

message("OPENNI_INCLUDEDIR : ${OPENNI_INCLUDEDIR}" )
message("OPENNI_LIBRARIES : ${OPENNI_LIBRARIES}" )

message("NITE2_INCLUDEDIR : ${NITE2_INCLUDEDIR}" )
message("NITE2_LIBRARIES : ${NITE2_LIBRARIES}" )

#export OPENNI2_INCLUDE=/home/jmainpri/Downloads/openni/OpenNI-2.1.0/Include
#export OPENNI2_REDIST=/home/jmainpri/Downloads/openni/OpenNI-2.1.0/Redist

#export NITE2_INCLUDE=/home/jmainpri/Downloads/openni/NiTE-2.0.0/Include
#export NITE2_REDIST64=/home/jmainpri/Downloads/openni/NiTE-2.0.0/Redist

include_directories(${OPENNI_INCLUDEDIR})
include_directories(${NITE2_INCLUDEDIR})
link_directories(/home/jmainpri/Downloads/openni/OpenNI2/Redist/OpenNI-2.1.0-x64/Redist/)
link_directories(/home/jmainpri/Downloads/openni/NiTE-2.0.0/Redist)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_executable(openni_tracker src/openni_tracker.cpp)
target_link_libraries(openni_tracker ${OPENNI_LIBRARIES} ${NITE2_LIBRARIES})
