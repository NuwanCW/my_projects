# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nuwan/projects/ros/robot/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nuwan/projects/ros/robot/catkin_ws/build

# Include any dependencies generated for this target.
include openslam_gmapping/CMakeFiles/configfile_test.dir/depend.make

# Include the progress variables for this target.
include openslam_gmapping/CMakeFiles/configfile_test.dir/progress.make

# Include the compile flags for this target's objects.
include openslam_gmapping/CMakeFiles/configfile_test.dir/flags.make

openslam_gmapping/CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.o: openslam_gmapping/CMakeFiles/configfile_test.dir/flags.make
openslam_gmapping/CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.o: /home/nuwan/projects/ros/robot/catkin_ws/src/openslam_gmapping/configfile/configfile_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nuwan/projects/ros/robot/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object openslam_gmapping/CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.o"
	cd /home/nuwan/projects/ros/robot/catkin_ws/build/openslam_gmapping && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.o -c /home/nuwan/projects/ros/robot/catkin_ws/src/openslam_gmapping/configfile/configfile_test.cpp

openslam_gmapping/CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.i"
	cd /home/nuwan/projects/ros/robot/catkin_ws/build/openslam_gmapping && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nuwan/projects/ros/robot/catkin_ws/src/openslam_gmapping/configfile/configfile_test.cpp > CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.i

openslam_gmapping/CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.s"
	cd /home/nuwan/projects/ros/robot/catkin_ws/build/openslam_gmapping && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nuwan/projects/ros/robot/catkin_ws/src/openslam_gmapping/configfile/configfile_test.cpp -o CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.s

openslam_gmapping/CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.o.requires:

.PHONY : openslam_gmapping/CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.o.requires

openslam_gmapping/CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.o.provides: openslam_gmapping/CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.o.requires
	$(MAKE) -f openslam_gmapping/CMakeFiles/configfile_test.dir/build.make openslam_gmapping/CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.o.provides.build
.PHONY : openslam_gmapping/CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.o.provides

openslam_gmapping/CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.o.provides.build: openslam_gmapping/CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.o


# Object files for target configfile_test
configfile_test_OBJECTS = \
"CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.o"

# External object files for target configfile_test
configfile_test_EXTERNAL_OBJECTS =

/home/nuwan/projects/ros/robot/catkin_ws/devel/lib/openslam_gmapping/configfile_test: openslam_gmapping/CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.o
/home/nuwan/projects/ros/robot/catkin_ws/devel/lib/openslam_gmapping/configfile_test: openslam_gmapping/CMakeFiles/configfile_test.dir/build.make
/home/nuwan/projects/ros/robot/catkin_ws/devel/lib/openslam_gmapping/configfile_test: /home/nuwan/projects/ros/robot/catkin_ws/devel/lib/libconfigfile.so
/home/nuwan/projects/ros/robot/catkin_ws/devel/lib/openslam_gmapping/configfile_test: openslam_gmapping/CMakeFiles/configfile_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nuwan/projects/ros/robot/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/nuwan/projects/ros/robot/catkin_ws/devel/lib/openslam_gmapping/configfile_test"
	cd /home/nuwan/projects/ros/robot/catkin_ws/build/openslam_gmapping && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/configfile_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
openslam_gmapping/CMakeFiles/configfile_test.dir/build: /home/nuwan/projects/ros/robot/catkin_ws/devel/lib/openslam_gmapping/configfile_test

.PHONY : openslam_gmapping/CMakeFiles/configfile_test.dir/build

openslam_gmapping/CMakeFiles/configfile_test.dir/requires: openslam_gmapping/CMakeFiles/configfile_test.dir/configfile/configfile_test.cpp.o.requires

.PHONY : openslam_gmapping/CMakeFiles/configfile_test.dir/requires

openslam_gmapping/CMakeFiles/configfile_test.dir/clean:
	cd /home/nuwan/projects/ros/robot/catkin_ws/build/openslam_gmapping && $(CMAKE_COMMAND) -P CMakeFiles/configfile_test.dir/cmake_clean.cmake
.PHONY : openslam_gmapping/CMakeFiles/configfile_test.dir/clean

openslam_gmapping/CMakeFiles/configfile_test.dir/depend:
	cd /home/nuwan/projects/ros/robot/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nuwan/projects/ros/robot/catkin_ws/src /home/nuwan/projects/ros/robot/catkin_ws/src/openslam_gmapping /home/nuwan/projects/ros/robot/catkin_ws/build /home/nuwan/projects/ros/robot/catkin_ws/build/openslam_gmapping /home/nuwan/projects/ros/robot/catkin_ws/build/openslam_gmapping/CMakeFiles/configfile_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : openslam_gmapping/CMakeFiles/configfile_test.dir/depend
