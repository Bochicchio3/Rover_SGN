# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/robot/catkin_ws/src/SerialManager

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robot/catkin_ws/build/serial_manager

# Include any dependencies generated for this target.
include CMakeFiles/SerialManager.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SerialManager.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SerialManager.dir/flags.make

CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.o: CMakeFiles/SerialManager.dir/flags.make
CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.o: /home/robot/catkin_ws/src/SerialManager/src/serial_manager_node_B.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/catkin_ws/build/serial_manager/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.o -c /home/robot/catkin_ws/src/SerialManager/src/serial_manager_node_B.cpp

CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robot/catkin_ws/src/SerialManager/src/serial_manager_node_B.cpp > CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.i

CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robot/catkin_ws/src/SerialManager/src/serial_manager_node_B.cpp -o CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.s

CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.o.requires:

.PHONY : CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.o.requires

CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.o.provides: CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.o.requires
	$(MAKE) -f CMakeFiles/SerialManager.dir/build.make CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.o.provides.build
.PHONY : CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.o.provides

CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.o.provides.build: CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.o


# Object files for target SerialManager
SerialManager_OBJECTS = \
"CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.o"

# External object files for target SerialManager
SerialManager_EXTERNAL_OBJECTS =

/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.o
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: CMakeFiles/SerialManager.dir/build.make
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /home/robot/catkin_ws/devel/.private/serial_manager/lib/libSerialManagerFunction.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /opt/ros/kinetic/lib/libroscpp.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /opt/ros/kinetic/lib/librosconsole.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /opt/ros/kinetic/lib/librostime.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /opt/ros/kinetic/lib/libcpp_common.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager: CMakeFiles/SerialManager.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot/catkin_ws/build/serial_manager/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SerialManager.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SerialManager.dir/build: /home/robot/catkin_ws/devel/.private/serial_manager/lib/serial_manager/SerialManager

.PHONY : CMakeFiles/SerialManager.dir/build

CMakeFiles/SerialManager.dir/requires: CMakeFiles/SerialManager.dir/src/serial_manager_node_B.cpp.o.requires

.PHONY : CMakeFiles/SerialManager.dir/requires

CMakeFiles/SerialManager.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SerialManager.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SerialManager.dir/clean

CMakeFiles/SerialManager.dir/depend:
	cd /home/robot/catkin_ws/build/serial_manager && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/catkin_ws/src/SerialManager /home/robot/catkin_ws/src/SerialManager /home/robot/catkin_ws/build/serial_manager /home/robot/catkin_ws/build/serial_manager /home/robot/catkin_ws/build/serial_manager/CMakeFiles/SerialManager.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SerialManager.dir/depend

