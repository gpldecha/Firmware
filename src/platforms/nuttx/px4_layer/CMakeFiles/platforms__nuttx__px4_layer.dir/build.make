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
include src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/depend.make

# Include the progress variables for this target.
include src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/progress.make

# Include the compile flags for this target's objects.
include src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/flags.make

src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.obj: src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/flags.make
src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.obj: src/platforms/nuttx/px4_layer/px4_nuttx_tasks.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.obj"
	cd /home/guillaume/src/Firmware/src/platforms/nuttx/px4_layer && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.obj   -c /home/guillaume/src/Firmware/src/platforms/nuttx/px4_layer/px4_nuttx_tasks.c

src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.i"
	cd /home/guillaume/src/Firmware/src/platforms/nuttx/px4_layer && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/guillaume/src/Firmware/src/platforms/nuttx/px4_layer/px4_nuttx_tasks.c > CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.i

src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.s"
	cd /home/guillaume/src/Firmware/src/platforms/nuttx/px4_layer && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/guillaume/src/Firmware/src/platforms/nuttx/px4_layer/px4_nuttx_tasks.c -o CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.s

src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.obj.requires:

.PHONY : src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.obj.requires

src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.obj.provides: src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.obj.requires
	$(MAKE) -f src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/build.make src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.obj.provides.build
.PHONY : src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.obj.provides

src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.obj.provides.build: src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.obj


src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.obj: src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/flags.make
src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.obj: src/platforms/posix/px4_layer/px4_log.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.obj"
	cd /home/guillaume/src/Firmware/src/platforms/nuttx/px4_layer && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.obj   -c /home/guillaume/src/Firmware/src/platforms/posix/px4_layer/px4_log.c

src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.i"
	cd /home/guillaume/src/Firmware/src/platforms/nuttx/px4_layer && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/guillaume/src/Firmware/src/platforms/posix/px4_layer/px4_log.c > CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.i

src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.s"
	cd /home/guillaume/src/Firmware/src/platforms/nuttx/px4_layer && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/guillaume/src/Firmware/src/platforms/posix/px4_layer/px4_log.c -o CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.s

src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.obj.requires:

.PHONY : src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.obj.requires

src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.obj.provides: src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.obj.requires
	$(MAKE) -f src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/build.make src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.obj.provides.build
.PHONY : src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.obj.provides

src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.obj.provides.build: src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.obj


# Object files for target platforms__nuttx__px4_layer
platforms__nuttx__px4_layer_OBJECTS = \
"CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.obj" \
"CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.obj"

# External object files for target platforms__nuttx__px4_layer
platforms__nuttx__px4_layer_EXTERNAL_OBJECTS =

src/platforms/nuttx/px4_layer/libplatforms__nuttx__px4_layer.a: src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.obj
src/platforms/nuttx/px4_layer/libplatforms__nuttx__px4_layer.a: src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.obj
src/platforms/nuttx/px4_layer/libplatforms__nuttx__px4_layer.a: src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/build.make
src/platforms/nuttx/px4_layer/libplatforms__nuttx__px4_layer.a: src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C static library libplatforms__nuttx__px4_layer.a"
	cd /home/guillaume/src/Firmware/src/platforms/nuttx/px4_layer && $(CMAKE_COMMAND) -P CMakeFiles/platforms__nuttx__px4_layer.dir/cmake_clean_target.cmake
	cd /home/guillaume/src/Firmware/src/platforms/nuttx/px4_layer && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/platforms__nuttx__px4_layer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/build: src/platforms/nuttx/px4_layer/libplatforms__nuttx__px4_layer.a

.PHONY : src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/build

src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/requires: src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/px4_nuttx_tasks.c.obj.requires
src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/requires: src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/__/__/posix/px4_layer/px4_log.c.obj.requires

.PHONY : src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/requires

src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/clean:
	cd /home/guillaume/src/Firmware/src/platforms/nuttx/px4_layer && $(CMAKE_COMMAND) -P CMakeFiles/platforms__nuttx__px4_layer.dir/cmake_clean.cmake
.PHONY : src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/clean

src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/depend:
	cd /home/guillaume/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/platforms/nuttx/px4_layer /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/platforms/nuttx/px4_layer /home/guillaume/src/Firmware/src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/platforms/nuttx/px4_layer/CMakeFiles/platforms__nuttx__px4_layer.dir/depend

