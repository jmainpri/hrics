if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing ")
endif()

#if (DEFINED ENV{ROS_ROOT})

#  cmake_minimum_required(VERSION 2.4.6)
#  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#  rosbuild_init()
#  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#  rosbuild_add_library(or-hrics main.cpp)
#  target_link_libraries(or-hrics)

#else (DEFINED ENV{ROS_ROOT})

  cmake_minimum_required (VERSION 2.4)
  if(POLICY CMP0015)
    cmake_policy(SET CMP0015 NEW)
  endif()
  project (or-hrics)

  message("CMAKE_SOURCE_DIR : " ${CMAKE_SOURCE_DIR})
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

  if( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )
    set(PLUGIN_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../plugins")
  else( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )
    set(PLUGIN_INSTALL_DIR "$ENV{PLUGIN_INSTALL_DIR}")
  endif( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )

  message("PLUGIN_INSTALL_DIR : " ${PLUGIN_INSTALL_DIR})

  # Find important packages
  find_package(OpenRAVE REQUIRED)
  message("OPENRAVE_INCLUDE_DIRS : " ${OPENRAVE_INCLUDE_DIRS})
  message("OPENRAVE_LIBRARY : " ${OPENRAVE_LIBRARY})
  #include(FindOpenRAVE.cmake)

  if( NOT OPENRAVE_FOUND )
    message(FATAL_ERROR "could not find openrave, check for openrave-config in path")
  endif( NOT OPENRAVE_FOUND )

  link_directories(${OPENRAVE_LINK_DIRS})
  add_library(or-hrics SHARED hrics-main.cpp)
  set_target_properties(or-hrics PROPERTIES
                        LINK_FLAGS ${OPENRAVE_LINK_FLAGS}
                        COMPILE_FLAGS ${OPENRAVE_CXXFLAGS})
  include_directories(${OPENRAVE_INCLUDE_DIRS})
  target_link_libraries(or-hrics ${OPENRAVE_LIBRARY})

  install(TARGETS or-hrics DESTINATION ${PLUGIN_INSTALL_DIR} )

#endif (DEFINED ENV{ROS_ROOT})
