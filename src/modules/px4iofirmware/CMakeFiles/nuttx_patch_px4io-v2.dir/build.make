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

# Utility rule file for nuttx_patch_px4io-v2.

# Include the progress variables for this target.
include src/modules/px4iofirmware/CMakeFiles/nuttx_patch_px4io-v2.dir/progress.make

nuttx_patch_px4io-v2: src/modules/px4iofirmware/CMakeFiles/nuttx_patch_px4io-v2.dir/build.make

.PHONY : nuttx_patch_px4io-v2

# Rule to build all files generated by this target.
src/modules/px4iofirmware/CMakeFiles/nuttx_patch_px4io-v2.dir/build: nuttx_patch_px4io-v2

.PHONY : src/modules/px4iofirmware/CMakeFiles/nuttx_patch_px4io-v2.dir/build

src/modules/px4iofirmware/CMakeFiles/nuttx_patch_px4io-v2.dir/clean:
	cd /home/guillaume/src/Firmware/src/modules/px4iofirmware && $(CMAKE_COMMAND) -P CMakeFiles/nuttx_patch_px4io-v2.dir/cmake_clean.cmake
.PHONY : src/modules/px4iofirmware/CMakeFiles/nuttx_patch_px4io-v2.dir/clean

src/modules/px4iofirmware/CMakeFiles/nuttx_patch_px4io-v2.dir/depend:
	cd /home/guillaume/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/modules/px4iofirmware /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/modules/px4iofirmware /home/guillaume/src/Firmware/src/modules/px4iofirmware/CMakeFiles/nuttx_patch_px4io-v2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/px4iofirmware/CMakeFiles/nuttx_patch_px4io-v2.dir/depend

