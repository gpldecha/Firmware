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
include src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/depend.make

# Include the progress variables for this target.
include src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/progress.make

# Include the compile flags for this target's objects.
include src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/flags.make

src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.obj: src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/flags.make
src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.obj: src/platforms/nuttx/px4_nuttx_impl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.obj"
	cd /home/guillaume/src/Firmware/src/platforms/nuttx && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.obj -c /home/guillaume/src/Firmware/src/platforms/nuttx/px4_nuttx_impl.cpp

src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.i"
	cd /home/guillaume/src/Firmware/src/platforms/nuttx && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guillaume/src/Firmware/src/platforms/nuttx/px4_nuttx_impl.cpp > CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.i

src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.s"
	cd /home/guillaume/src/Firmware/src/platforms/nuttx && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guillaume/src/Firmware/src/platforms/nuttx/px4_nuttx_impl.cpp -o CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.s

src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.obj.requires:

.PHONY : src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.obj.requires

src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.obj.provides: src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.obj.requires
	$(MAKE) -f src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/build.make src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.obj.provides.build
.PHONY : src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.obj.provides

src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.obj.provides.build: src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.obj


# Object files for target platforms__nuttx
platforms__nuttx_OBJECTS = \
"CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.obj"

# External object files for target platforms__nuttx
platforms__nuttx_EXTERNAL_OBJECTS =

src/platforms/nuttx/libplatforms__nuttx.a: src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.obj
src/platforms/nuttx/libplatforms__nuttx.a: src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/build.make
src/platforms/nuttx/libplatforms__nuttx.a: src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libplatforms__nuttx.a"
	cd /home/guillaume/src/Firmware/src/platforms/nuttx && $(CMAKE_COMMAND) -P CMakeFiles/platforms__nuttx.dir/cmake_clean_target.cmake
	cd /home/guillaume/src/Firmware/src/platforms/nuttx && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/platforms__nuttx.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/build: src/platforms/nuttx/libplatforms__nuttx.a

.PHONY : src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/build

src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/requires: src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/px4_nuttx_impl.cpp.obj.requires

.PHONY : src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/requires

src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/clean:
	cd /home/guillaume/src/Firmware/src/platforms/nuttx && $(CMAKE_COMMAND) -P CMakeFiles/platforms__nuttx.dir/cmake_clean.cmake
.PHONY : src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/clean

src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/depend:
	cd /home/guillaume/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/platforms/nuttx /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/platforms/nuttx /home/guillaume/src/Firmware/src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/platforms/nuttx/CMakeFiles/platforms__nuttx.dir/depend

