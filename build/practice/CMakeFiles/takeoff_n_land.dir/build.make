# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/user/uav_work/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/uav_work/build

# Include any dependencies generated for this target.
include practice/CMakeFiles/takeoff_n_land.dir/depend.make

# Include the progress variables for this target.
include practice/CMakeFiles/takeoff_n_land.dir/progress.make

# Include the compile flags for this target's objects.
include practice/CMakeFiles/takeoff_n_land.dir/flags.make

practice/CMakeFiles/takeoff_n_land.dir/src/takeoff_n_land.cpp.o: practice/CMakeFiles/takeoff_n_land.dir/flags.make
practice/CMakeFiles/takeoff_n_land.dir/src/takeoff_n_land.cpp.o: /home/user/uav_work/src/practice/src/takeoff_n_land.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/uav_work/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object practice/CMakeFiles/takeoff_n_land.dir/src/takeoff_n_land.cpp.o"
	cd /home/user/uav_work/build/practice && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/takeoff_n_land.dir/src/takeoff_n_land.cpp.o -c /home/user/uav_work/src/practice/src/takeoff_n_land.cpp

practice/CMakeFiles/takeoff_n_land.dir/src/takeoff_n_land.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/takeoff_n_land.dir/src/takeoff_n_land.cpp.i"
	cd /home/user/uav_work/build/practice && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/uav_work/src/practice/src/takeoff_n_land.cpp > CMakeFiles/takeoff_n_land.dir/src/takeoff_n_land.cpp.i

practice/CMakeFiles/takeoff_n_land.dir/src/takeoff_n_land.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/takeoff_n_land.dir/src/takeoff_n_land.cpp.s"
	cd /home/user/uav_work/build/practice && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/uav_work/src/practice/src/takeoff_n_land.cpp -o CMakeFiles/takeoff_n_land.dir/src/takeoff_n_land.cpp.s

# Object files for target takeoff_n_land
takeoff_n_land_OBJECTS = \
"CMakeFiles/takeoff_n_land.dir/src/takeoff_n_land.cpp.o"

# External object files for target takeoff_n_land
takeoff_n_land_EXTERNAL_OBJECTS =

/home/user/uav_work/devel/lib/practice/takeoff_n_land: practice/CMakeFiles/takeoff_n_land.dir/src/takeoff_n_land.cpp.o
/home/user/uav_work/devel/lib/practice/takeoff_n_land: practice/CMakeFiles/takeoff_n_land.dir/build.make
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /opt/ros/noetic/lib/libroscpp.so
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /opt/ros/noetic/lib/librosconsole.so
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /opt/ros/noetic/lib/librostime.so
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /opt/ros/noetic/lib/libcpp_common.so
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/user/uav_work/devel/lib/practice/takeoff_n_land: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/user/uav_work/devel/lib/practice/takeoff_n_land: practice/CMakeFiles/takeoff_n_land.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/uav_work/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/user/uav_work/devel/lib/practice/takeoff_n_land"
	cd /home/user/uav_work/build/practice && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/takeoff_n_land.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
practice/CMakeFiles/takeoff_n_land.dir/build: /home/user/uav_work/devel/lib/practice/takeoff_n_land

.PHONY : practice/CMakeFiles/takeoff_n_land.dir/build

practice/CMakeFiles/takeoff_n_land.dir/clean:
	cd /home/user/uav_work/build/practice && $(CMAKE_COMMAND) -P CMakeFiles/takeoff_n_land.dir/cmake_clean.cmake
.PHONY : practice/CMakeFiles/takeoff_n_land.dir/clean

practice/CMakeFiles/takeoff_n_land.dir/depend:
	cd /home/user/uav_work/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/uav_work/src /home/user/uav_work/src/practice /home/user/uav_work/build /home/user/uav_work/build/practice /home/user/uav_work/build/practice/CMakeFiles/takeoff_n_land.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : practice/CMakeFiles/takeoff_n_land.dir/depend

