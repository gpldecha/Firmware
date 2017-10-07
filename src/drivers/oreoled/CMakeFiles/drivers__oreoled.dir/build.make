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
include src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/depend.make

# Include the progress variables for this target.
include src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/flags.make

src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/oreoled.cpp.obj: src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/flags.make
src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/oreoled.cpp.obj: src/drivers/oreoled/oreoled.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/oreoled.cpp.obj"
	cd /home/guillaume/src/Firmware/src/drivers/oreoled && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__oreoled.dir/oreoled.cpp.obj -c /home/guillaume/src/Firmware/src/drivers/oreoled/oreoled.cpp

src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/oreoled.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__oreoled.dir/oreoled.cpp.i"
	cd /home/guillaume/src/Firmware/src/drivers/oreoled && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guillaume/src/Firmware/src/drivers/oreoled/oreoled.cpp > CMakeFiles/drivers__oreoled.dir/oreoled.cpp.i

src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/oreoled.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__oreoled.dir/oreoled.cpp.s"
	cd /home/guillaume/src/Firmware/src/drivers/oreoled && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guillaume/src/Firmware/src/drivers/oreoled/oreoled.cpp -o CMakeFiles/drivers__oreoled.dir/oreoled.cpp.s

src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/oreoled.cpp.obj.requires:

.PHONY : src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/oreoled.cpp.obj.requires

src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/oreoled.cpp.obj.provides: src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/oreoled.cpp.obj.requires
	$(MAKE) -f src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/build.make src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/oreoled.cpp.obj.provides.build
.PHONY : src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/oreoled.cpp.obj.provides

src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/oreoled.cpp.obj.provides.build: src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/oreoled.cpp.obj


# Object files for target drivers__oreoled
drivers__oreoled_OBJECTS = \
"CMakeFiles/drivers__oreoled.dir/oreoled.cpp.obj"

# External object files for target drivers__oreoled
drivers__oreoled_EXTERNAL_OBJECTS =

src/drivers/oreoled/libdrivers__oreoled.a: src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/oreoled.cpp.obj
src/drivers/oreoled/libdrivers__oreoled.a: src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/build.make
src/drivers/oreoled/libdrivers__oreoled.a: src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libdrivers__oreoled.a"
	cd /home/guillaume/src/Firmware/src/drivers/oreoled && $(CMAKE_COMMAND) -P CMakeFiles/drivers__oreoled.dir/cmake_clean_target.cmake
	cd /home/guillaume/src/Firmware/src/drivers/oreoled && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__oreoled.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/build: src/drivers/oreoled/libdrivers__oreoled.a

.PHONY : src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/build

src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/requires: src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/oreoled.cpp.obj.requires

.PHONY : src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/requires

src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/clean:
	cd /home/guillaume/src/Firmware/src/drivers/oreoled && $(CMAKE_COMMAND) -P CMakeFiles/drivers__oreoled.dir/cmake_clean.cmake
.PHONY : src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/clean

src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/depend:
	cd /home/guillaume/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/drivers/oreoled /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/drivers/oreoled /home/guillaume/src/Firmware/src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/oreoled/CMakeFiles/drivers__oreoled.dir/depend

