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
include src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/depend.make

# Include the progress variables for this target.
include src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/flags.make

src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/px4flow.cpp.obj: src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/flags.make
src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/px4flow.cpp.obj: src/drivers/px4flow/px4flow.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/px4flow.cpp.obj"
	cd /home/guillaume/src/Firmware/src/drivers/px4flow && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__px4flow.dir/px4flow.cpp.obj -c /home/guillaume/src/Firmware/src/drivers/px4flow/px4flow.cpp

src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/px4flow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__px4flow.dir/px4flow.cpp.i"
	cd /home/guillaume/src/Firmware/src/drivers/px4flow && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guillaume/src/Firmware/src/drivers/px4flow/px4flow.cpp > CMakeFiles/drivers__px4flow.dir/px4flow.cpp.i

src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/px4flow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__px4flow.dir/px4flow.cpp.s"
	cd /home/guillaume/src/Firmware/src/drivers/px4flow && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guillaume/src/Firmware/src/drivers/px4flow/px4flow.cpp -o CMakeFiles/drivers__px4flow.dir/px4flow.cpp.s

src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/px4flow.cpp.obj.requires:

.PHONY : src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/px4flow.cpp.obj.requires

src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/px4flow.cpp.obj.provides: src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/px4flow.cpp.obj.requires
	$(MAKE) -f src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/build.make src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/px4flow.cpp.obj.provides.build
.PHONY : src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/px4flow.cpp.obj.provides

src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/px4flow.cpp.obj.provides.build: src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/px4flow.cpp.obj


# Object files for target drivers__px4flow
drivers__px4flow_OBJECTS = \
"CMakeFiles/drivers__px4flow.dir/px4flow.cpp.obj"

# External object files for target drivers__px4flow
drivers__px4flow_EXTERNAL_OBJECTS =

src/drivers/px4flow/libdrivers__px4flow.a: src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/px4flow.cpp.obj
src/drivers/px4flow/libdrivers__px4flow.a: src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/build.make
src/drivers/px4flow/libdrivers__px4flow.a: src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libdrivers__px4flow.a"
	cd /home/guillaume/src/Firmware/src/drivers/px4flow && $(CMAKE_COMMAND) -P CMakeFiles/drivers__px4flow.dir/cmake_clean_target.cmake
	cd /home/guillaume/src/Firmware/src/drivers/px4flow && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__px4flow.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/build: src/drivers/px4flow/libdrivers__px4flow.a

.PHONY : src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/build

src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/requires: src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/px4flow.cpp.obj.requires

.PHONY : src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/requires

src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/clean:
	cd /home/guillaume/src/Firmware/src/drivers/px4flow && $(CMAKE_COMMAND) -P CMakeFiles/drivers__px4flow.dir/cmake_clean.cmake
.PHONY : src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/clean

src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/depend:
	cd /home/guillaume/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/drivers/px4flow /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/drivers/px4flow /home/guillaume/src/Firmware/src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/px4flow/CMakeFiles/drivers__px4flow.dir/depend

