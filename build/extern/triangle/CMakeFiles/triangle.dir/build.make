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
include extern/triangle/CMakeFiles/triangle.dir/depend.make

# Include the progress variables for this target.
include extern/triangle/CMakeFiles/triangle.dir/progress.make

# Include the compile flags for this target's objects.
include extern/triangle/CMakeFiles/triangle.dir/flags.make

extern/triangle/CMakeFiles/triangle.dir/triangle.c.o: extern/triangle/CMakeFiles/triangle.dir/flags.make
extern/triangle/CMakeFiles/triangle.dir/triangle.c.o: ../extern/triangle/triangle.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object extern/triangle/CMakeFiles/triangle.dir/triangle.c.o"
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/extern/triangle && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/triangle.dir/triangle.c.o   -c /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/extern/triangle/triangle.c

extern/triangle/CMakeFiles/triangle.dir/triangle.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/triangle.dir/triangle.c.i"
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/extern/triangle && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/extern/triangle/triangle.c > CMakeFiles/triangle.dir/triangle.c.i

extern/triangle/CMakeFiles/triangle.dir/triangle.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/triangle.dir/triangle.c.s"
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/extern/triangle && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/extern/triangle/triangle.c -o CMakeFiles/triangle.dir/triangle.c.s

# Object files for target triangle
triangle_OBJECTS = \
"CMakeFiles/triangle.dir/triangle.c.o"

# External object files for target triangle
triangle_EXTERNAL_OBJECTS =

../lib/release/libtriangle.a: extern/triangle/CMakeFiles/triangle.dir/triangle.c.o
../lib/release/libtriangle.a: extern/triangle/CMakeFiles/triangle.dir/build.make
../lib/release/libtriangle.a: extern/triangle/CMakeFiles/triangle.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library ../../../lib/release/libtriangle.a"
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/extern/triangle && $(CMAKE_COMMAND) -P CMakeFiles/triangle.dir/cmake_clean_target.cmake
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/extern/triangle && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/triangle.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
extern/triangle/CMakeFiles/triangle.dir/build: ../lib/release/libtriangle.a

.PHONY : extern/triangle/CMakeFiles/triangle.dir/build

extern/triangle/CMakeFiles/triangle.dir/clean:
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/extern/triangle && $(CMAKE_COMMAND) -P CMakeFiles/triangle.dir/cmake_clean.cmake
.PHONY : extern/triangle/CMakeFiles/triangle.dir/clean

extern/triangle/CMakeFiles/triangle.dir/depend:
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/extern/triangle /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/extern/triangle /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/extern/triangle/CMakeFiles/triangle.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : extern/triangle/CMakeFiles/triangle.dir/depend
