# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.17.0_1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.17.0_1/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/xuzhizun/Documents/datacollect/tools

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/xuzhizun/Documents/datacollect/tools/build

# Include any dependencies generated for this target.
include CMakeFiles/synchronize_single.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/synchronize_single.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/synchronize_single.dir/flags.make

CMakeFiles/synchronize_single.dir/associate_single.cpp.o: CMakeFiles/synchronize_single.dir/flags.make
CMakeFiles/synchronize_single.dir/associate_single.cpp.o: ../associate_single.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/xuzhizun/Documents/datacollect/tools/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/synchronize_single.dir/associate_single.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/synchronize_single.dir/associate_single.cpp.o -c /Users/xuzhizun/Documents/datacollect/tools/associate_single.cpp

CMakeFiles/synchronize_single.dir/associate_single.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/synchronize_single.dir/associate_single.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/xuzhizun/Documents/datacollect/tools/associate_single.cpp > CMakeFiles/synchronize_single.dir/associate_single.cpp.i

CMakeFiles/synchronize_single.dir/associate_single.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/synchronize_single.dir/associate_single.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/xuzhizun/Documents/datacollect/tools/associate_single.cpp -o CMakeFiles/synchronize_single.dir/associate_single.cpp.s

# Object files for target synchronize_single
synchronize_single_OBJECTS = \
"CMakeFiles/synchronize_single.dir/associate_single.cpp.o"

# External object files for target synchronize_single
synchronize_single_EXTERNAL_OBJECTS =

synchronize_single: CMakeFiles/synchronize_single.dir/associate_single.cpp.o
synchronize_single: CMakeFiles/synchronize_single.dir/build.make
synchronize_single: CMakeFiles/synchronize_single.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/xuzhizun/Documents/datacollect/tools/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable synchronize_single"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/synchronize_single.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/synchronize_single.dir/build: synchronize_single

.PHONY : CMakeFiles/synchronize_single.dir/build

CMakeFiles/synchronize_single.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/synchronize_single.dir/cmake_clean.cmake
.PHONY : CMakeFiles/synchronize_single.dir/clean

CMakeFiles/synchronize_single.dir/depend:
	cd /Users/xuzhizun/Documents/datacollect/tools/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/xuzhizun/Documents/datacollect/tools /Users/xuzhizun/Documents/datacollect/tools /Users/xuzhizun/Documents/datacollect/tools/build /Users/xuzhizun/Documents/datacollect/tools/build /Users/xuzhizun/Documents/datacollect/tools/build/CMakeFiles/synchronize_single.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/synchronize_single.dir/depend

