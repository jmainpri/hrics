# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rafihayne/workspace/hrics-or-plugins/openni_tracker_multi

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rafihayne/workspace/hrics-or-plugins/openni_tracker_multi/build

# Include any dependencies generated for this target.
include CMakeFiles/tracker1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/tracker1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tracker1.dir/flags.make

CMakeFiles/tracker1.dir/src/openni_tracker1.o: CMakeFiles/tracker1.dir/flags.make
CMakeFiles/tracker1.dir/src/openni_tracker1.o: ../src/openni_tracker1.cpp
CMakeFiles/tracker1.dir/src/openni_tracker1.o: ../manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/stacks/orocos_kinematics_dynamics/orocos_kdl/manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/stacks/orocos_kinematics_dynamics/python_orocos_kdl/manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/stacks/orocos_kinematics_dynamics/kdl/manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/tracker1.dir/src/openni_tracker1.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rafihayne/workspace/hrics-or-plugins/openni_tracker_multi/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/tracker1.dir/src/openni_tracker1.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/tracker1.dir/src/openni_tracker1.o -c /home/rafihayne/workspace/hrics-or-plugins/openni_tracker_multi/src/openni_tracker1.cpp

CMakeFiles/tracker1.dir/src/openni_tracker1.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracker1.dir/src/openni_tracker1.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/rafihayne/workspace/hrics-or-plugins/openni_tracker_multi/src/openni_tracker1.cpp > CMakeFiles/tracker1.dir/src/openni_tracker1.i

CMakeFiles/tracker1.dir/src/openni_tracker1.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracker1.dir/src/openni_tracker1.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/rafihayne/workspace/hrics-or-plugins/openni_tracker_multi/src/openni_tracker1.cpp -o CMakeFiles/tracker1.dir/src/openni_tracker1.s

CMakeFiles/tracker1.dir/src/openni_tracker1.o.requires:
.PHONY : CMakeFiles/tracker1.dir/src/openni_tracker1.o.requires

CMakeFiles/tracker1.dir/src/openni_tracker1.o.provides: CMakeFiles/tracker1.dir/src/openni_tracker1.o.requires
	$(MAKE) -f CMakeFiles/tracker1.dir/build.make CMakeFiles/tracker1.dir/src/openni_tracker1.o.provides.build
.PHONY : CMakeFiles/tracker1.dir/src/openni_tracker1.o.provides

CMakeFiles/tracker1.dir/src/openni_tracker1.o.provides.build: CMakeFiles/tracker1.dir/src/openni_tracker1.o

# Object files for target tracker1
tracker1_OBJECTS = \
"CMakeFiles/tracker1.dir/src/openni_tracker1.o"

# External object files for target tracker1
tracker1_EXTERNAL_OBJECTS =

../bin/tracker1: CMakeFiles/tracker1.dir/src/openni_tracker1.o
../bin/tracker1: CMakeFiles/tracker1.dir/build.make
../bin/tracker1: CMakeFiles/tracker1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/tracker1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tracker1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/tracker1.dir/build: ../bin/tracker1
.PHONY : CMakeFiles/tracker1.dir/build

CMakeFiles/tracker1.dir/requires: CMakeFiles/tracker1.dir/src/openni_tracker1.o.requires
.PHONY : CMakeFiles/tracker1.dir/requires

CMakeFiles/tracker1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tracker1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tracker1.dir/clean

CMakeFiles/tracker1.dir/depend:
	cd /home/rafihayne/workspace/hrics-or-plugins/openni_tracker_multi/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rafihayne/workspace/hrics-or-plugins/openni_tracker_multi /home/rafihayne/workspace/hrics-or-plugins/openni_tracker_multi /home/rafihayne/workspace/hrics-or-plugins/openni_tracker_multi/build /home/rafihayne/workspace/hrics-or-plugins/openni_tracker_multi/build /home/rafihayne/workspace/hrics-or-plugins/openni_tracker_multi/build/CMakeFiles/tracker1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tracker1.dir/depend
