# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.18.4/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.18.4/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/xuzhizun/Git_ws/datacollect/tools

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/xuzhizun/Git_ws/datacollect/tools/build_mac

# Include any dependencies generated for this target.
include CMakeFiles/synchronize_stereo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/synchronize_stereo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/synchronize_stereo.dir/flags.make

CMakeFiles/synchronize_stereo.dir/associate_stereo.cpp.o: CMakeFiles/synchronize_stereo.dir/flags.make
CMakeFiles/synchronize_stereo.dir/associate_stereo.cpp.o: ../associate_stereo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/xuzhizun/Git_ws/datacollect/tools/build_mac/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/synchronize_stereo.dir/associate_stereo.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/synchronize_stereo.dir/associate_stereo.cpp.o -c /Users/xuzhizun/Git_ws/datacollect/tools/associate_stereo.cpp

CMakeFiles/synchronize_stereo.dir/associate_stereo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/synchronize_stereo.dir/associate_stereo.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/xuzhizun/Git_ws/datacollect/tools/associate_stereo.cpp > CMakeFiles/synchronize_stereo.dir/associate_stereo.cpp.i

CMakeFiles/synchronize_stereo.dir/associate_stereo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/synchronize_stereo.dir/associate_stereo.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/xuzhizun/Git_ws/datacollect/tools/associate_stereo.cpp -o CMakeFiles/synchronize_stereo.dir/associate_stereo.cpp.s

# Object files for target synchronize_stereo
synchronize_stereo_OBJECTS = \
"CMakeFiles/synchronize_stereo.dir/associate_stereo.cpp.o"

# External object files for target synchronize_stereo
synchronize_stereo_EXTERNAL_OBJECTS =

synchronize_stereo: CMakeFiles/synchronize_stereo.dir/associate_stereo.cpp.o
synchronize_stereo: CMakeFiles/synchronize_stereo.dir/build.make
synchronize_stereo: CMakeFiles/synchronize_stereo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/xuzhizun/Git_ws/datacollect/tools/build_mac/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable synchronize_stereo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/synchronize_stereo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/synchronize_stereo.dir/build: synchronize_stereo

.PHONY : CMakeFiles/synchronize_stereo.dir/build

CMakeFiles/synchronize_stereo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/synchronize_stereo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/synchronize_stereo.dir/clean

CMakeFiles/synchronize_stereo.dir/depend:
	cd /Users/xuzhizun/Git_ws/datacollect/tools/build_mac && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/xuzhizun/Git_ws/datacollect/tools /Users/xuzhizun/Git_ws/datacollect/tools /Users/xuzhizun/Git_ws/datacollect/tools/build_mac /Users/xuzhizun/Git_ws/datacollect/tools/build_mac /Users/xuzhizun/Git_ws/datacollect/tools/build_mac/CMakeFiles/synchronize_stereo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/synchronize_stereo.dir/depend
