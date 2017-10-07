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
include src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/depend.make

# Include the progress variables for this target.
include src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/flags.make

src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.obj: src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/flags.make
src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.obj: src/drivers/tap_esc/tap_esc.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.obj"
	cd /home/guillaume/src/Firmware/src/drivers/tap_esc && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.obj -c /home/guillaume/src/Firmware/src/drivers/tap_esc/tap_esc.cpp

src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.i"
	cd /home/guillaume/src/Firmware/src/drivers/tap_esc && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guillaume/src/Firmware/src/drivers/tap_esc/tap_esc.cpp > CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.i

src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.s"
	cd /home/guillaume/src/Firmware/src/drivers/tap_esc && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guillaume/src/Firmware/src/drivers/tap_esc/tap_esc.cpp -o CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.s

src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.obj.requires:

.PHONY : src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.obj.requires

src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.obj.provides: src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.obj.requires
	$(MAKE) -f src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/build.make src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.obj.provides.build
.PHONY : src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.obj.provides

src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.obj.provides.build: src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.obj


src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.obj: src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/flags.make
src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.obj: src/drivers/tap_esc/tap_esc_common.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.obj"
	cd /home/guillaume/src/Firmware/src/drivers/tap_esc && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.obj -c /home/guillaume/src/Firmware/src/drivers/tap_esc/tap_esc_common.cpp

src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.i"
	cd /home/guillaume/src/Firmware/src/drivers/tap_esc && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guillaume/src/Firmware/src/drivers/tap_esc/tap_esc_common.cpp > CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.i

src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.s"
	cd /home/guillaume/src/Firmware/src/drivers/tap_esc && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guillaume/src/Firmware/src/drivers/tap_esc/tap_esc_common.cpp -o CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.s

src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.obj.requires:

.PHONY : src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.obj.requires

src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.obj.provides: src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.obj.requires
	$(MAKE) -f src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/build.make src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.obj.provides.build
.PHONY : src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.obj.provides

src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.obj.provides.build: src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.obj


# Object files for target drivers__tap_esc
drivers__tap_esc_OBJECTS = \
"CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.obj" \
"CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.obj"

# External object files for target drivers__tap_esc
drivers__tap_esc_EXTERNAL_OBJECTS =

src/drivers/tap_esc/libdrivers__tap_esc.a: src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.obj
src/drivers/tap_esc/libdrivers__tap_esc.a: src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.obj
src/drivers/tap_esc/libdrivers__tap_esc.a: src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/build.make
src/drivers/tap_esc/libdrivers__tap_esc.a: src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libdrivers__tap_esc.a"
	cd /home/guillaume/src/Firmware/src/drivers/tap_esc && $(CMAKE_COMMAND) -P CMakeFiles/drivers__tap_esc.dir/cmake_clean_target.cmake
	cd /home/guillaume/src/Firmware/src/drivers/tap_esc && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__tap_esc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/build: src/drivers/tap_esc/libdrivers__tap_esc.a

.PHONY : src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/build

src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/requires: src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc.cpp.obj.requires
src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/requires: src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/tap_esc_common.cpp.obj.requires

.PHONY : src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/requires

src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/clean:
	cd /home/guillaume/src/Firmware/src/drivers/tap_esc && $(CMAKE_COMMAND) -P CMakeFiles/drivers__tap_esc.dir/cmake_clean.cmake
.PHONY : src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/clean

src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/depend:
	cd /home/guillaume/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/drivers/tap_esc /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/drivers/tap_esc /home/guillaume/src/Firmware/src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/tap_esc/CMakeFiles/drivers__tap_esc.dir/depend
