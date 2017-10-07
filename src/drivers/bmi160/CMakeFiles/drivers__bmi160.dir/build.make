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
include src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/depend.make

# Include the progress variables for this target.
include src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/flags.make

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160.cpp.obj: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/flags.make
src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160.cpp.obj: src/drivers/bmi160/bmi160.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160.cpp.obj"
	cd /home/guillaume/src/Firmware/src/drivers/bmi160 && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__bmi160.dir/bmi160.cpp.obj -c /home/guillaume/src/Firmware/src/drivers/bmi160/bmi160.cpp

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__bmi160.dir/bmi160.cpp.i"
	cd /home/guillaume/src/Firmware/src/drivers/bmi160 && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guillaume/src/Firmware/src/drivers/bmi160/bmi160.cpp > CMakeFiles/drivers__bmi160.dir/bmi160.cpp.i

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__bmi160.dir/bmi160.cpp.s"
	cd /home/guillaume/src/Firmware/src/drivers/bmi160 && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guillaume/src/Firmware/src/drivers/bmi160/bmi160.cpp -o CMakeFiles/drivers__bmi160.dir/bmi160.cpp.s

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160.cpp.obj.requires:

.PHONY : src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160.cpp.obj.requires

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160.cpp.obj.provides: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160.cpp.obj.requires
	$(MAKE) -f src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/build.make src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160.cpp.obj.provides.build
.PHONY : src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160.cpp.obj.provides

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160.cpp.obj.provides.build: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160.cpp.obj


src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.obj: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/flags.make
src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.obj: src/drivers/bmi160/bmi160_gyro.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.obj"
	cd /home/guillaume/src/Firmware/src/drivers/bmi160 && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.obj -c /home/guillaume/src/Firmware/src/drivers/bmi160/bmi160_gyro.cpp

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.i"
	cd /home/guillaume/src/Firmware/src/drivers/bmi160 && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guillaume/src/Firmware/src/drivers/bmi160/bmi160_gyro.cpp > CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.i

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.s"
	cd /home/guillaume/src/Firmware/src/drivers/bmi160 && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guillaume/src/Firmware/src/drivers/bmi160/bmi160_gyro.cpp -o CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.s

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.obj.requires:

.PHONY : src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.obj.requires

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.obj.provides: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.obj.requires
	$(MAKE) -f src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/build.make src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.obj.provides.build
.PHONY : src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.obj.provides

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.obj.provides.build: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.obj


src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.obj: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/flags.make
src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.obj: src/drivers/bmi160/bmi160_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.obj"
	cd /home/guillaume/src/Firmware/src/drivers/bmi160 && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.obj -c /home/guillaume/src/Firmware/src/drivers/bmi160/bmi160_main.cpp

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.i"
	cd /home/guillaume/src/Firmware/src/drivers/bmi160 && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guillaume/src/Firmware/src/drivers/bmi160/bmi160_main.cpp > CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.i

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.s"
	cd /home/guillaume/src/Firmware/src/drivers/bmi160 && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guillaume/src/Firmware/src/drivers/bmi160/bmi160_main.cpp -o CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.s

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.obj.requires:

.PHONY : src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.obj.requires

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.obj.provides: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.obj.requires
	$(MAKE) -f src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/build.make src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.obj.provides.build
.PHONY : src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.obj.provides

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.obj.provides.build: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.obj


# Object files for target drivers__bmi160
drivers__bmi160_OBJECTS = \
"CMakeFiles/drivers__bmi160.dir/bmi160.cpp.obj" \
"CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.obj" \
"CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.obj"

# External object files for target drivers__bmi160
drivers__bmi160_EXTERNAL_OBJECTS =

src/drivers/bmi160/libdrivers__bmi160.a: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160.cpp.obj
src/drivers/bmi160/libdrivers__bmi160.a: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.obj
src/drivers/bmi160/libdrivers__bmi160.a: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.obj
src/drivers/bmi160/libdrivers__bmi160.a: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/build.make
src/drivers/bmi160/libdrivers__bmi160.a: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libdrivers__bmi160.a"
	cd /home/guillaume/src/Firmware/src/drivers/bmi160 && $(CMAKE_COMMAND) -P CMakeFiles/drivers__bmi160.dir/cmake_clean_target.cmake
	cd /home/guillaume/src/Firmware/src/drivers/bmi160 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__bmi160.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/build: src/drivers/bmi160/libdrivers__bmi160.a

.PHONY : src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/build

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/requires: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160.cpp.obj.requires
src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/requires: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_gyro.cpp.obj.requires
src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/requires: src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/bmi160_main.cpp.obj.requires

.PHONY : src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/requires

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/clean:
	cd /home/guillaume/src/Firmware/src/drivers/bmi160 && $(CMAKE_COMMAND) -P CMakeFiles/drivers__bmi160.dir/cmake_clean.cmake
.PHONY : src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/clean

src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/depend:
	cd /home/guillaume/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/drivers/bmi160 /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/drivers/bmi160 /home/guillaume/src/Firmware/src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/bmi160/CMakeFiles/drivers__bmi160.dir/depend

