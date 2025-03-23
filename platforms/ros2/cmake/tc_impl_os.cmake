############################################################################
#
# Copyright (c) 2022 TC Development Team. All rights reserved.
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
# 3. Neither the name TC nor the names of its contributors may be
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

#=============================================================================
#
#	Defined functions in this file
#
# 	OS Specific Functions
#
#		* tc_posix_generate_builtin_commands
#
# 	Required OS Interface Functions
#
# 		* tc_os_add_flags
# 		* tc_os_determine_build_chip
#		* tc_os_prebuild_targets
#

#=============================================================================
#
#	tc_posix_generate_builtin_commands
#
#	This function generates the builtin_commands.c src for posix
#
#	Usage:
#		tc_posix_generate_builtin_commands(
#			MODULE_LIST <in-list>
#			OUT <file>)
#
#	Input:
#		MODULE_LIST	: list of modules
#
#	Output:
#		OUT	: stem of generated apps.cpp/apps.h ("apps")
#
#	Example:
#		tc_posix_generate_builtin_commands(
#			OUT <generated-src> MODULE_LIST tc_simple_app)
#
function(tc_posix_generate_builtin_commands)
	tc_parse_function_args(
		NAME tc_posix_generate_builtin_commands
		ONE_VALUE OUT
		MULTI_VALUE MODULE_LIST
		REQUIRED MODULE_LIST OUT
		ARGN ${ARGN})

endfunction()

#=============================================================================
#
#	tc_os_add_flags
#
#	Set the posix build flags.
#
#	Usage:
#		tc_os_add_flags()
#
function(tc_os_add_flags)

	add_definitions(
		-D__TC_ROS2
		-D__TC_POSIX
		-Dnoreturn_function=__attribute__\(\(noreturn\)\)
	)

	include_directories(platforms/ros2/include)

endfunction()

#=============================================================================
#
#	tc_os_determine_build_chip
#
#	Sets TC_CHIP and TC_CHIP_MANUFACTURER.
#
#	Usage:
#		tc_os_determine_build_chip()
#
function(tc_os_determine_build_chip)

	# always use generic chip and chip manufacturer
	set(TC_CHIP "generic" CACHE STRING "TC Chip" FORCE)
	set(TC_CHIP_MANUFACTURER "generic" CACHE STRING "TC Chip Manufacturer" FORCE)

endfunction()

#=============================================================================
#
#	tc_os_prebuild_targets
#
#	This function generates os dependent targets
#
#	Usage:
#		tc_os_prebuild_targets(
#			OUT <out-list_of_targets>
#			BOARD <in-string>
#			)
#
#	Input:
#		BOARD		: board
#
#	Output:
#		OUT	: the target list
#
#	Example:
#		tc_os_prebuild_targets(OUT target_list BOARD tc_fmu-v2)
#
function(tc_os_prebuild_targets)
	tc_parse_function_args(
			NAME tc_os_prebuild_targets
			ONE_VALUE OUT BOARD
			REQUIRED OUT
			ARGN ${ARGN})

	add_library(prebuild_targets INTERFACE)

endfunction()
