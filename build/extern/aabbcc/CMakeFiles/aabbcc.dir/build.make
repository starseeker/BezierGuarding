# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /opt/local/bin/cmake

# The command to remove a file.
RM = /opt/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build

# Include any dependencies generated for this target.
include extern/aabbcc/CMakeFiles/aabbcc.dir/depend.make

# Include the progress variables for this target.
include extern/aabbcc/CMakeFiles/aabbcc.dir/progress.make

# Include the compile flags for this target's objects.
include extern/aabbcc/CMakeFiles/aabbcc.dir/flags.make

extern/aabbcc/CMakeFiles/aabbcc.dir/AABB.cc.o: extern/aabbcc/CMakeFiles/aabbcc.dir/flags.make
extern/aabbcc/CMakeFiles/aabbcc.dir/AABB.cc.o: ../extern/aabbcc/AABB.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object extern/aabbcc/CMakeFiles/aabbcc.dir/AABB.cc.o"
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/extern/aabbcc && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aabbcc.dir/AABB.cc.o -c /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/extern/aabbcc/AABB.cc

extern/aabbcc/CMakeFiles/aabbcc.dir/AABB.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aabbcc.dir/AABB.cc.i"
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/extern/aabbcc && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/extern/aabbcc/AABB.cc > CMakeFiles/aabbcc.dir/AABB.cc.i

extern/aabbcc/CMakeFiles/aabbcc.dir/AABB.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aabbcc.dir/AABB.cc.s"
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/extern/aabbcc && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/extern/aabbcc/AABB.cc -o CMakeFiles/aabbcc.dir/AABB.cc.s

# Object files for target aabbcc
aabbcc_OBJECTS = \
"CMakeFiles/aabbcc.dir/AABB.cc.o"

# External object files for target aabbcc
aabbcc_EXTERNAL_OBJECTS =

../lib/release/libaabbcc.a: extern/aabbcc/CMakeFiles/aabbcc.dir/AABB.cc.o
../lib/release/libaabbcc.a: extern/aabbcc/CMakeFiles/aabbcc.dir/build.make
../lib/release/libaabbcc.a: extern/aabbcc/CMakeFiles/aabbcc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../../../lib/release/libaabbcc.a"
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/extern/aabbcc && $(CMAKE_COMMAND) -P CMakeFiles/aabbcc.dir/cmake_clean_target.cmake
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/extern/aabbcc && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aabbcc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
extern/aabbcc/CMakeFiles/aabbcc.dir/build: ../lib/release/libaabbcc.a

.PHONY : extern/aabbcc/CMakeFiles/aabbcc.dir/build

extern/aabbcc/CMakeFiles/aabbcc.dir/clean:
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/extern/aabbcc && $(CMAKE_COMMAND) -P CMakeFiles/aabbcc.dir/cmake_clean.cmake
.PHONY : extern/aabbcc/CMakeFiles/aabbcc.dir/clean

extern/aabbcc/CMakeFiles/aabbcc.dir/depend:
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/extern/aabbcc /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/extern/aabbcc /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/extern/aabbcc/CMakeFiles/aabbcc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : extern/aabbcc/CMakeFiles/aabbcc.dir/depend

