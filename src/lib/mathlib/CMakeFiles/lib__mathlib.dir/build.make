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
CMAKE_SOURCE_DIR = /home/guillaume/src/Firmware

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/guillaume/src/Firmware

# Include any dependencies generated for this target.
include src/lib/mathlib/CMakeFiles/lib__mathlib.dir/depend.make

# Include the progress variables for this target.
include src/lib/mathlib/CMakeFiles/lib__mathlib.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/mathlib/CMakeFiles/lib__mathlib.dir/flags.make

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/test/test.cpp.obj: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/flags.make
src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/test/test.cpp.obj: src/lib/mathlib/math/test/test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/test/test.cpp.obj"
	cd /home/guillaume/src/Firmware/src/lib/mathlib && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib__mathlib.dir/math/test/test.cpp.obj -c /home/guillaume/src/Firmware/src/lib/mathlib/math/test/test.cpp

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/test/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib__mathlib.dir/math/test/test.cpp.i"
	cd /home/guillaume/src/Firmware/src/lib/mathlib && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guillaume/src/Firmware/src/lib/mathlib/math/test/test.cpp > CMakeFiles/lib__mathlib.dir/math/test/test.cpp.i

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/test/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib__mathlib.dir/math/test/test.cpp.s"
	cd /home/guillaume/src/Firmware/src/lib/mathlib && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guillaume/src/Firmware/src/lib/mathlib/math/test/test.cpp -o CMakeFiles/lib__mathlib.dir/math/test/test.cpp.s

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/test/test.cpp.obj.requires:

.PHONY : src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/test/test.cpp.obj.requires

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/test/test.cpp.obj.provides: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/test/test.cpp.obj.requires
	$(MAKE) -f src/lib/mathlib/CMakeFiles/lib__mathlib.dir/build.make src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/test/test.cpp.obj.provides.build
.PHONY : src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/test/test.cpp.obj.provides

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/test/test.cpp.obj.provides.build: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/test/test.cpp.obj


src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/Limits.cpp.obj: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/flags.make
src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/Limits.cpp.obj: src/lib/mathlib/math/Limits.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/Limits.cpp.obj"
	cd /home/guillaume/src/Firmware/src/lib/mathlib && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib__mathlib.dir/math/Limits.cpp.obj -c /home/guillaume/src/Firmware/src/lib/mathlib/math/Limits.cpp

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/Limits.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib__mathlib.dir/math/Limits.cpp.i"
	cd /home/guillaume/src/Firmware/src/lib/mathlib && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guillaume/src/Firmware/src/lib/mathlib/math/Limits.cpp > CMakeFiles/lib__mathlib.dir/math/Limits.cpp.i

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/Limits.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib__mathlib.dir/math/Limits.cpp.s"
	cd /home/guillaume/src/Firmware/src/lib/mathlib && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guillaume/src/Firmware/src/lib/mathlib/math/Limits.cpp -o CMakeFiles/lib__mathlib.dir/math/Limits.cpp.s

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/Limits.cpp.obj.requires:

.PHONY : src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/Limits.cpp.obj.requires

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/Limits.cpp.obj.provides: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/Limits.cpp.obj.requires
	$(MAKE) -f src/lib/mathlib/CMakeFiles/lib__mathlib.dir/build.make src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/Limits.cpp.obj.provides.build
.PHONY : src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/Limits.cpp.obj.provides

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/Limits.cpp.obj.provides.build: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/Limits.cpp.obj


src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.obj: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/flags.make
src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.obj: src/lib/mathlib/math/matrix_alg.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.obj"
	cd /home/guillaume/src/Firmware/src/lib/mathlib && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.obj -c /home/guillaume/src/Firmware/src/lib/mathlib/math/matrix_alg.cpp

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.i"
	cd /home/guillaume/src/Firmware/src/lib/mathlib && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guillaume/src/Firmware/src/lib/mathlib/math/matrix_alg.cpp > CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.i

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.s"
	cd /home/guillaume/src/Firmware/src/lib/mathlib && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guillaume/src/Firmware/src/lib/mathlib/math/matrix_alg.cpp -o CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.s

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.obj.requires:

.PHONY : src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.obj.requires

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.obj.provides: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.obj.requires
	$(MAKE) -f src/lib/mathlib/CMakeFiles/lib__mathlib.dir/build.make src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.obj.provides.build
.PHONY : src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.obj.provides

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.obj.provides.build: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.obj


# Object files for target lib__mathlib
lib__mathlib_OBJECTS = \
"CMakeFiles/lib__mathlib.dir/math/test/test.cpp.obj" \
"CMakeFiles/lib__mathlib.dir/math/Limits.cpp.obj" \
"CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.obj"

# External object files for target lib__mathlib
lib__mathlib_EXTERNAL_OBJECTS =

src/lib/mathlib/liblib__mathlib.a: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/test/test.cpp.obj
src/lib/mathlib/liblib__mathlib.a: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/Limits.cpp.obj
src/lib/mathlib/liblib__mathlib.a: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.obj
src/lib/mathlib/liblib__mathlib.a: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/build.make
src/lib/mathlib/liblib__mathlib.a: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library liblib__mathlib.a"
	cd /home/guillaume/src/Firmware/src/lib/mathlib && $(CMAKE_COMMAND) -P CMakeFiles/lib__mathlib.dir/cmake_clean_target.cmake
	cd /home/guillaume/src/Firmware/src/lib/mathlib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lib__mathlib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/mathlib/CMakeFiles/lib__mathlib.dir/build: src/lib/mathlib/liblib__mathlib.a

.PHONY : src/lib/mathlib/CMakeFiles/lib__mathlib.dir/build

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/requires: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/test/test.cpp.obj.requires
src/lib/mathlib/CMakeFiles/lib__mathlib.dir/requires: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/Limits.cpp.obj.requires
src/lib/mathlib/CMakeFiles/lib__mathlib.dir/requires: src/lib/mathlib/CMakeFiles/lib__mathlib.dir/math/matrix_alg.cpp.obj.requires

.PHONY : src/lib/mathlib/CMakeFiles/lib__mathlib.dir/requires

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/clean:
	cd /home/guillaume/src/Firmware/src/lib/mathlib && $(CMAKE_COMMAND) -P CMakeFiles/lib__mathlib.dir/cmake_clean.cmake
.PHONY : src/lib/mathlib/CMakeFiles/lib__mathlib.dir/clean

src/lib/mathlib/CMakeFiles/lib__mathlib.dir/depend:
	cd /home/guillaume/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/lib/mathlib /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/lib/mathlib /home/guillaume/src/Firmware/src/lib/mathlib/CMakeFiles/lib__mathlib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/mathlib/CMakeFiles/lib__mathlib.dir/depend

