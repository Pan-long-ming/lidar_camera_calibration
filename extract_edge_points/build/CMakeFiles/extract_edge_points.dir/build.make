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
CMAKE_SOURCE_DIR = /home/pan/projects/extract_edge_points

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pan/projects/extract_edge_points/build

# Include any dependencies generated for this target.
include CMakeFiles/extract_edge_points.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/extract_edge_points.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/extract_edge_points.dir/flags.make

CMakeFiles/extract_edge_points.dir/main.cpp.o: CMakeFiles/extract_edge_points.dir/flags.make
CMakeFiles/extract_edge_points.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pan/projects/extract_edge_points/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/extract_edge_points.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/extract_edge_points.dir/main.cpp.o -c /home/pan/projects/extract_edge_points/main.cpp

CMakeFiles/extract_edge_points.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/extract_edge_points.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pan/projects/extract_edge_points/main.cpp > CMakeFiles/extract_edge_points.dir/main.cpp.i

CMakeFiles/extract_edge_points.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/extract_edge_points.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pan/projects/extract_edge_points/main.cpp -o CMakeFiles/extract_edge_points.dir/main.cpp.s

CMakeFiles/extract_edge_points.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/extract_edge_points.dir/main.cpp.o.requires

CMakeFiles/extract_edge_points.dir/main.cpp.o.provides: CMakeFiles/extract_edge_points.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/extract_edge_points.dir/build.make CMakeFiles/extract_edge_points.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/extract_edge_points.dir/main.cpp.o.provides

CMakeFiles/extract_edge_points.dir/main.cpp.o.provides.build: CMakeFiles/extract_edge_points.dir/main.cpp.o


# Object files for target extract_edge_points
extract_edge_points_OBJECTS = \
"CMakeFiles/extract_edge_points.dir/main.cpp.o"

# External object files for target extract_edge_points
extract_edge_points_EXTERNAL_OBJECTS =

extract_edge_points: CMakeFiles/extract_edge_points.dir/main.cpp.o
extract_edge_points: CMakeFiles/extract_edge_points.dir/build.make
extract_edge_points: CMakeFiles/extract_edge_points.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pan/projects/extract_edge_points/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable extract_edge_points"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/extract_edge_points.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/extract_edge_points.dir/build: extract_edge_points

.PHONY : CMakeFiles/extract_edge_points.dir/build

CMakeFiles/extract_edge_points.dir/requires: CMakeFiles/extract_edge_points.dir/main.cpp.o.requires

.PHONY : CMakeFiles/extract_edge_points.dir/requires

CMakeFiles/extract_edge_points.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/extract_edge_points.dir/cmake_clean.cmake
.PHONY : CMakeFiles/extract_edge_points.dir/clean

CMakeFiles/extract_edge_points.dir/depend:
	cd /home/pan/projects/extract_edge_points/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pan/projects/extract_edge_points /home/pan/projects/extract_edge_points /home/pan/projects/extract_edge_points/build /home/pan/projects/extract_edge_points/build /home/pan/projects/extract_edge_points/build/CMakeFiles/extract_edge_points.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/extract_edge_points.dir/depend
