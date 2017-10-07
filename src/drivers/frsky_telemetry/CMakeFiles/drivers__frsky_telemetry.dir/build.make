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
include src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/depend.make

# Include the progress variables for this target.
include src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/flags.make

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.obj: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/flags.make
src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.obj: src/drivers/frsky_telemetry/frsky_data.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.obj"
	cd /home/guillaume/src/Firmware/src/drivers/frsky_telemetry && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.obj   -c /home/guillaume/src/Firmware/src/drivers/frsky_telemetry/frsky_data.c

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.i"
	cd /home/guillaume/src/Firmware/src/drivers/frsky_telemetry && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/guillaume/src/Firmware/src/drivers/frsky_telemetry/frsky_data.c > CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.i

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.s"
	cd /home/guillaume/src/Firmware/src/drivers/frsky_telemetry && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/guillaume/src/Firmware/src/drivers/frsky_telemetry/frsky_data.c -o CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.s

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.obj.requires:

.PHONY : src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.obj.requires

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.obj.provides: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.obj.requires
	$(MAKE) -f src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/build.make src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.obj.provides.build
.PHONY : src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.obj.provides

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.obj.provides.build: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.obj


src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.obj: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/flags.make
src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.obj: src/drivers/frsky_telemetry/sPort_data.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.obj"
	cd /home/guillaume/src/Firmware/src/drivers/frsky_telemetry && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.obj   -c /home/guillaume/src/Firmware/src/drivers/frsky_telemetry/sPort_data.c

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.i"
	cd /home/guillaume/src/Firmware/src/drivers/frsky_telemetry && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/guillaume/src/Firmware/src/drivers/frsky_telemetry/sPort_data.c > CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.i

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.s"
	cd /home/guillaume/src/Firmware/src/drivers/frsky_telemetry && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/guillaume/src/Firmware/src/drivers/frsky_telemetry/sPort_data.c -o CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.s

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.obj.requires:

.PHONY : src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.obj.requires

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.obj.provides: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.obj.requires
	$(MAKE) -f src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/build.make src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.obj.provides.build
.PHONY : src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.obj.provides

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.obj.provides.build: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.obj


src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.obj: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/flags.make
src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.obj: src/drivers/frsky_telemetry/frsky_telemetry.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.obj"
	cd /home/guillaume/src/Firmware/src/drivers/frsky_telemetry && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.obj   -c /home/guillaume/src/Firmware/src/drivers/frsky_telemetry/frsky_telemetry.c

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.i"
	cd /home/guillaume/src/Firmware/src/drivers/frsky_telemetry && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/guillaume/src/Firmware/src/drivers/frsky_telemetry/frsky_telemetry.c > CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.i

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.s"
	cd /home/guillaume/src/Firmware/src/drivers/frsky_telemetry && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/guillaume/src/Firmware/src/drivers/frsky_telemetry/frsky_telemetry.c -o CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.s

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.obj.requires:

.PHONY : src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.obj.requires

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.obj.provides: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.obj.requires
	$(MAKE) -f src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/build.make src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.obj.provides.build
.PHONY : src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.obj.provides

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.obj.provides.build: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.obj


# Object files for target drivers__frsky_telemetry
drivers__frsky_telemetry_OBJECTS = \
"CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.obj" \
"CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.obj" \
"CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.obj"

# External object files for target drivers__frsky_telemetry
drivers__frsky_telemetry_EXTERNAL_OBJECTS =

src/drivers/frsky_telemetry/libdrivers__frsky_telemetry.a: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.obj
src/drivers/frsky_telemetry/libdrivers__frsky_telemetry.a: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.obj
src/drivers/frsky_telemetry/libdrivers__frsky_telemetry.a: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.obj
src/drivers/frsky_telemetry/libdrivers__frsky_telemetry.a: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/build.make
src/drivers/frsky_telemetry/libdrivers__frsky_telemetry.a: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking C static library libdrivers__frsky_telemetry.a"
	cd /home/guillaume/src/Firmware/src/drivers/frsky_telemetry && $(CMAKE_COMMAND) -P CMakeFiles/drivers__frsky_telemetry.dir/cmake_clean_target.cmake
	cd /home/guillaume/src/Firmware/src/drivers/frsky_telemetry && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__frsky_telemetry.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/build: src/drivers/frsky_telemetry/libdrivers__frsky_telemetry.a

.PHONY : src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/build

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/requires: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.c.obj.requires
src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/requires: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.c.obj.requires
src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/requires: src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.c.obj.requires

.PHONY : src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/requires

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/clean:
	cd /home/guillaume/src/Firmware/src/drivers/frsky_telemetry && $(CMAKE_COMMAND) -P CMakeFiles/drivers__frsky_telemetry.dir/cmake_clean.cmake
.PHONY : src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/clean

src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/depend:
	cd /home/guillaume/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/drivers/frsky_telemetry /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/drivers/frsky_telemetry /home/guillaume/src/Firmware/src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/depend

