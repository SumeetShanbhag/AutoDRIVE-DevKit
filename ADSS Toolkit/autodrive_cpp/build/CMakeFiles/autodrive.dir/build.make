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
CMAKE_SOURCE_DIR = "/home/sumeet/catkin_ws/src/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_cpp"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/sumeet/catkin_ws/src/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_cpp/build"

# Include any dependencies generated for this target.
include CMakeFiles/autodrive.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/autodrive.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/autodrive.dir/flags.make

CMakeFiles/autodrive.dir/src/autodrive.cpp.o: CMakeFiles/autodrive.dir/flags.make
CMakeFiles/autodrive.dir/src/autodrive.cpp.o: ../src/autodrive.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/sumeet/catkin_ws/src/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_cpp/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/autodrive.dir/src/autodrive.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/autodrive.dir/src/autodrive.cpp.o -c "/home/sumeet/catkin_ws/src/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_cpp/src/autodrive.cpp"

CMakeFiles/autodrive.dir/src/autodrive.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/autodrive.dir/src/autodrive.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/sumeet/catkin_ws/src/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_cpp/src/autodrive.cpp" > CMakeFiles/autodrive.dir/src/autodrive.cpp.i

CMakeFiles/autodrive.dir/src/autodrive.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/autodrive.dir/src/autodrive.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/sumeet/catkin_ws/src/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_cpp/src/autodrive.cpp" -o CMakeFiles/autodrive.dir/src/autodrive.cpp.s

# Object files for target autodrive
autodrive_OBJECTS = \
"CMakeFiles/autodrive.dir/src/autodrive.cpp.o"

# External object files for target autodrive
autodrive_EXTERNAL_OBJECTS =

libautodrive.a: CMakeFiles/autodrive.dir/src/autodrive.cpp.o
libautodrive.a: CMakeFiles/autodrive.dir/build.make
libautodrive.a: CMakeFiles/autodrive.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/sumeet/catkin_ws/src/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_cpp/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libautodrive.a"
	$(CMAKE_COMMAND) -P CMakeFiles/autodrive.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/autodrive.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/autodrive.dir/build: libautodrive.a

.PHONY : CMakeFiles/autodrive.dir/build

CMakeFiles/autodrive.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/autodrive.dir/cmake_clean.cmake
.PHONY : CMakeFiles/autodrive.dir/clean

CMakeFiles/autodrive.dir/depend:
	cd "/home/sumeet/catkin_ws/src/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_cpp/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/sumeet/catkin_ws/src/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_cpp" "/home/sumeet/catkin_ws/src/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_cpp" "/home/sumeet/catkin_ws/src/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_cpp/build" "/home/sumeet/catkin_ws/src/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_cpp/build" "/home/sumeet/catkin_ws/src/AutoDRIVE-Devkit/ADSS Toolkit/autodrive_cpp/build/CMakeFiles/autodrive.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/autodrive.dir/depend

