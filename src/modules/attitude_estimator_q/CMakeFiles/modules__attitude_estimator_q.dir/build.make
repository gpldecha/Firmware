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
include src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/depend.make

# Include the progress variables for this target.
include src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/progress.make

# Include the compile flags for this target's objects.
include src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/flags.make

src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.obj: src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/flags.make
src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.obj: src/modules/attitude_estimator_q/attitude_estimator_q_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.obj"
	cd /home/guillaume/src/Firmware/src/modules/attitude_estimator_q && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.obj -c /home/guillaume/src/Firmware/src/modules/attitude_estimator_q/attitude_estimator_q_main.cpp

src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.i"
	cd /home/guillaume/src/Firmware/src/modules/attitude_estimator_q && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guillaume/src/Firmware/src/modules/attitude_estimator_q/attitude_estimator_q_main.cpp > CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.i

src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.s"
	cd /home/guillaume/src/Firmware/src/modules/attitude_estimator_q && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guillaume/src/Firmware/src/modules/attitude_estimator_q/attitude_estimator_q_main.cpp -o CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.s

src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.obj.requires:

.PHONY : src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.obj.requires

src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.obj.provides: src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.obj.requires
	$(MAKE) -f src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/build.make src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.obj.provides.build
.PHONY : src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.obj.provides

src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.obj.provides.build: src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.obj


# Object files for target modules__attitude_estimator_q
modules__attitude_estimator_q_OBJECTS = \
"CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.obj"

# External object files for target modules__attitude_estimator_q
modules__attitude_estimator_q_EXTERNAL_OBJECTS =

src/modules/attitude_estimator_q/libmodules__attitude_estimator_q.a: src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.obj
src/modules/attitude_estimator_q/libmodules__attitude_estimator_q.a: src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/build.make
src/modules/attitude_estimator_q/libmodules__attitude_estimator_q.a: src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libmodules__attitude_estimator_q.a"
	cd /home/guillaume/src/Firmware/src/modules/attitude_estimator_q && $(CMAKE_COMMAND) -P CMakeFiles/modules__attitude_estimator_q.dir/cmake_clean_target.cmake
	cd /home/guillaume/src/Firmware/src/modules/attitude_estimator_q && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/modules__attitude_estimator_q.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/build: src/modules/attitude_estimator_q/libmodules__attitude_estimator_q.a

.PHONY : src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/build

src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/requires: src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/attitude_estimator_q_main.cpp.obj.requires

.PHONY : src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/requires

src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/clean:
	cd /home/guillaume/src/Firmware/src/modules/attitude_estimator_q && $(CMAKE_COMMAND) -P CMakeFiles/modules__attitude_estimator_q.dir/cmake_clean.cmake
.PHONY : src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/clean

src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/depend:
	cd /home/guillaume/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/modules/attitude_estimator_q /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/modules/attitude_estimator_q /home/guillaume/src/Firmware/src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/attitude_estimator_q/CMakeFiles/modules__attitude_estimator_q.dir/depend

