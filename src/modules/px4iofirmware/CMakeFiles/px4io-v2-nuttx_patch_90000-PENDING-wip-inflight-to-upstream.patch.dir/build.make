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

# Utility rule file for px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch.

# Include the progress variables for this target.
include src/modules/px4iofirmware/CMakeFiles/px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch.dir/progress.make

src/modules/px4iofirmware/CMakeFiles/px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch: px4io-v2/NuttX/px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch.stamp


px4io-v2/NuttX/px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch.stamp: nuttx-patches/90000-PENDING-wip-inflight-to-upstream.patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/guillaume/src/Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "px4io-v2: nuttx-patches/90000-PENDING-wip-inflight-to-upstream.patch applied"
	cd /home/guillaume/src/Firmware/src/modules/px4iofirmware && /usr/bin/patch --verbose -d /home/guillaume/src/Firmware/px4io-v2/NuttX -s -p1 -N < /home/guillaume/src/Firmware/nuttx-patches/90000-PENDING-wip-inflight-to-upstream.patch
	cd /home/guillaume/src/Firmware/src/modules/px4iofirmware && cmake -E touch /home/guillaume/src/Firmware/px4io-v2/NuttX/px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch.stamp

px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch: src/modules/px4iofirmware/CMakeFiles/px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch
px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch: px4io-v2/NuttX/px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch.stamp
px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch: src/modules/px4iofirmware/CMakeFiles/px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch.dir/build.make

.PHONY : px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch

# Rule to build all files generated by this target.
src/modules/px4iofirmware/CMakeFiles/px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch.dir/build: px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch

.PHONY : src/modules/px4iofirmware/CMakeFiles/px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch.dir/build

src/modules/px4iofirmware/CMakeFiles/px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch.dir/clean:
	cd /home/guillaume/src/Firmware/src/modules/px4iofirmware && $(CMAKE_COMMAND) -P CMakeFiles/px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch.dir/cmake_clean.cmake
.PHONY : src/modules/px4iofirmware/CMakeFiles/px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch.dir/clean

src/modules/px4iofirmware/CMakeFiles/px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch.dir/depend:
	cd /home/guillaume/src/Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/modules/px4iofirmware /home/guillaume/src/Firmware /home/guillaume/src/Firmware/src/modules/px4iofirmware /home/guillaume/src/Firmware/src/modules/px4iofirmware/CMakeFiles/px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/px4iofirmware/CMakeFiles/px4io-v2-nuttx_patch_90000-PENDING-wip-inflight-to-upstream.patch.dir/depend

