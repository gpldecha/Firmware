############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
set(PATH_COMMANDER "../commander")

include_directories(${PATH_COMMANDER})

px4_add_module(
	MODULE modules__rover_commander
	MAIN commander
	STACK_MAIN 4096
	STACK_MAX 2450
	COMPILE_FLAGS
        INCLUDES
                rover_commander
	SRCS
                rover_commander.cpp
		${PATH_COMMANDER}/state_machine_helper.cpp
		${PATH_COMMANDER}/commander_helper.cpp
		${PATH_COMMANDER}/calibration_routines.cpp
		${PATH_COMMANDER}/accelerometer_calibration.cpp
		${PATH_COMMANDER}/gyro_calibration.cpp
		${PATH_COMMANDER}/mag_calibration.cpp
		${PATH_COMMANDER}/baro_calibration.cpp
		${PATH_COMMANDER}/rc_calibration.cpp
		${PATH_COMMANDER}/airspeed_calibration.cpp
		${PATH_COMMANDER}/esc_calibration.cpp
		${PATH_COMMANDER}/PreflightCheck.cpp
	DEPENDS
		platforms__common

	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
