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
include apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/depend.make

# Include the progress variables for this target.
include apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/progress.make

# Include the compile flags for this target's objects.
include apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/flags.make

apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/main.cpp.o: apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/flags.make
apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/main.cpp.o: ../apps/bezmeshCLI/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/main.cpp.o"
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/apps/bezmeshCLI && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bezmeshCLI.dir/main.cpp.o -c /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/apps/bezmeshCLI/main.cpp

apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bezmeshCLI.dir/main.cpp.i"
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/apps/bezmeshCLI && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/apps/bezmeshCLI/main.cpp > CMakeFiles/bezmeshCLI.dir/main.cpp.i

apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bezmeshCLI.dir/main.cpp.s"
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/apps/bezmeshCLI && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/apps/bezmeshCLI/main.cpp -o CMakeFiles/bezmeshCLI.dir/main.cpp.s

# Object files for target bezmeshCLI
bezmeshCLI_OBJECTS = \
"CMakeFiles/bezmeshCLI.dir/main.cpp.o"

# External object files for target bezmeshCLI
bezmeshCLI_EXTERNAL_OBJECTS =

../bin/release/bezmeshCLI: apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/main.cpp.o
../bin/release/bezmeshCLI: apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/build.make
../bin/release/bezmeshCLI: ../lib/release/libbezmesh.a
../bin/release/bezmeshCLI: ../lib/release/libtriangle.a
../bin/release/bezmeshCLI: /opt/local/lib/libgmpxx.dylib
../bin/release/bezmeshCLI: /opt/local/lib/libCGAL.13.0.2.dylib
../bin/release/bezmeshCLI: /opt/local/lib/libmpfr.dylib
../bin/release/bezmeshCLI: /opt/local/lib/libgmp.dylib
../bin/release/bezmeshCLI: /opt/local/lib/libboost_thread-mt.dylib
../bin/release/bezmeshCLI: /opt/local/lib/libboost_system-mt.dylib
../bin/release/bezmeshCLI: /opt/local/lib/libboost_chrono-mt.dylib
../bin/release/bezmeshCLI: /opt/local/lib/libboost_date_time-mt.dylib
../bin/release/bezmeshCLI: /opt/local/lib/libboost_atomic-mt.dylib
../bin/release/bezmeshCLI: /opt/local/lib/libboost_thread-mt.dylib
../bin/release/bezmeshCLI: /opt/local/lib/libboost_chrono-mt.dylib
../bin/release/bezmeshCLI: /opt/local/lib/libboost_system-mt.dylib
../bin/release/bezmeshCLI: /opt/local/lib/libboost_date_time-mt.dylib
../bin/release/bezmeshCLI: /opt/local/lib/libboost_atomic-mt.dylib
../bin/release/bezmeshCLI: ../lib/release/libaabbcc.a
../bin/release/bezmeshCLI: apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/release/bezmeshCLI"
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/apps/bezmeshCLI && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bezmeshCLI.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/build: ../bin/release/bezmeshCLI

.PHONY : apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/build

apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/clean:
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/apps/bezmeshCLI && $(CMAKE_COMMAND) -P CMakeFiles/bezmeshCLI.dir/cmake_clean.cmake
.PHONY : apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/clean

apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/depend:
	cd /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/apps/bezmeshCLI /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/apps/bezmeshCLI /Users/campen/Documents/Papers/BezierGuarding/code_git/BezierGuarding/build/apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/bezmeshCLI/CMakeFiles/bezmeshCLI.dir/depend
