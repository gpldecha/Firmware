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
include src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/depend.make

# Include the progress variables for this target.
include src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/flags.make

src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/rgbled.cpp.obj: src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/flags.make
src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/rgbled.cpp.obj: src/drivers/rgbled/rgbled.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/rgbled.cpp.obj"
	cd /home/guillaume/src/Firmware/src/drivers/rgbled && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__rgbled.dir/rgbled.cpp.obj -c /home/guillaume/src/Firmware/src/drivers/rgbled/rgbled.cpp

src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/rgbled.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__rgbled.dir/rgbled.cpp.i"
	cd /home/guillaume/src/Firmware/src/drivers/rgbled && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guillaume/src/Firmware/src/drivers/rgbled/rgbled.cpp > CMakeFiles/drivers__rgbled.dir/rgbled.cpp.i

src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/rgbled.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__rgbled.dir/rgbled.cpp.s"
	cd /home/guillaume/src/Firmware/src/drivers/rgbled && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guillaume/src/Firmware/src/drivers/rgbled/rgbled.cpp -o CMakeFiles/drivers__rgbled.dir/rgbled.cpp.s

src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/rgbled.cpp.obj.requires:

.PHONY : src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/rgbled.cpp.obj.requires

src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/rgbled.cpp.obj.provides: src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/rgbled.cpp.obj.requires
	$(MAKE) -f src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/build.make src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/rgbled.cpp.obj.provides.build
.PHONY : src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/rgbled.cpp.obj.provides

src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/rgbled.cpp.obj.provides.build: src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/rgbled.cpp.obj


# Object files for target drivers__rgbled
drivers__rgbled_OBJECTS = \
"CMakeFiles/drivers__rgbled.dir/rgbled.cpp.obj"

# External object files for target drivers__rgbled
drivers__rgbled_EXTERNAL_OBJECTS =

src/drivers/rgbled/libdrivers__rgbled.a: src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/rgbled.cpp.obj
src/drivers/rgbled/libdrivers__rgbled.a: src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/build.make
src/drivers/rgbled/libdrivers__rgbled.a: src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libdrivers__rgbled.a"
	cd /home/guillaume/src/Firmware/src/drivers/rgbled && $(CMAKE_COMMAND) -P CMakeFiles/drivers__rgbled.dir/cmake_clean_target.cmake
	cd /home/guillaume/src/Firmware/src/drivers/rgbled && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__rgbled.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/build: src/drivers/rgbled/libdrivers__rgbled.a

.PHONY : src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/build

src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/requires: src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/rgbled.cpp.obj.requires

.PHONY : src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/requires

src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/clean:
	cd /home/guillaume/src/Firmware/src/drivers/rgbled && $(CMAKE_COMMAND) -P CMakeFiles/drivers__rgbled.dir/cmake_clean.cmake
.PHONY : src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/clean

src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/depend:
	cd /home/guillaume/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/drivers/rgbled /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/drivers/rgbled /home/guillaume/src/Firmware/src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/rgbled/CMakeFiles/drivers__rgbled.dir/depend

