############################################################################
#
# Copyright (c) 2015 TC Development Team. All rights reserved.
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

	set(builtin_apps_string)
	set(builtin_apps_decl_string)
	set(command_count 0)
	foreach(module ${MODULE_LIST})
		# default
		set(MAIN_DEFAULT MAIN-NOTFOUND)
		set(STACK_DEFAULT 1024)
		set(PRIORITY_DEFAULT SCHED_PRIORITY_DEFAULT)
		foreach(property MAIN STACK PRIORITY)
			get_target_property(${property} ${module} ${property})
			if(NOT ${property})
				set(${property} ${${property}_DEFAULT})
			endif()
		endforeach()
		if (MAIN)
			set(builtin_apps_string
				"${builtin_apps_string}\tapps[\"${MAIN}\"] = ${MAIN}_main;\n")
			set(builtin_apps_decl_string
				"${builtin_apps_decl_string}int ${MAIN}_main(int argc, char *argv[]);\n")
			math(EXPR command_count "${command_count}+1")
		endif()
	endforeach()
	configure_file(${TC_SOURCE_DIR}/platforms/common/apps.cpp.in ${OUT}.cpp)
	configure_file(${TC_SOURCE_DIR}/platforms/common/apps.h.in ${OUT}.h)
endfunction()


#=============================================================================
#
#	tc_posix_generate_alias
#
#	This function generates the tc-alias.sh script containing the command
#	aliases for all modules and commands.
#
#	Usage:
#		tc_posix_generate_alias(
#			MODULE_LIST <in-list>
#			OUT <file>
#			PREFIX <prefix>)
#
#	Input:
#		MODULE_LIST	: list of modules
#		PREFIX	: command prefix (e.g. "tc-")
#
#	Output:
#		OUT	: tc-alias.sh file path
#
#	Example:
#		tc_posix_generate_alias(
#			OUT <generated-src> MODULE_LIST tc_simple_app PREFIX tc-)
#
function(tc_posix_generate_alias)
	tc_parse_function_args(
		NAME tc_posix_generate_alias
		ONE_VALUE OUT PREFIX
		MULTI_VALUE MODULE_LIST
		REQUIRED OUT PREFIX MODULE_LIST
		ARGN ${ARGN})

	set(alias_string)
	foreach(module ${MODULE_LIST})
		foreach(property MAIN STACK PRIORITY)
			get_target_property(${property} ${module} ${property})
			if(NOT ${property})
				set(${property} ${${property}_DEFAULT})
			endif()
		endforeach()
		if (MAIN)
			set(alias_string
				"${alias_string}alias ${MAIN}='${PREFIX}${MAIN} --instance $tc_instance'\n"
			)
		endif()
	endforeach()
	configure_file(${TC_SOURCE_DIR}/platforms/posix/src/tc/common/tc-alias.sh_in ${OUT})
endfunction()


#=============================================================================
#
#	tc_posix_generate_symlinks
#
#	This function generates symlinks for all modules/commands.
#
#	Usage:
#		tc_posix_generate_symlinks(
#			TARGET <target>
#			MODULE_LIST <in-list>
#			PREFIX <prefix>)
#
#	Input:
#		MODULE_LIST	: list of modules
#		PREFIX	: command prefix (e.g. "tc-")
#		TARGET	: cmake target for which the symlinks should be created
#
#	Example:
#		tc_posix_generate_symlinks(
#			TARGET tc MODULE_LIST tc_simple_app PREFIX tc-)
#
function(tc_posix_generate_symlinks)
	tc_parse_function_args(
		NAME tc_posix_generate_symlinks
		ONE_VALUE TARGET PREFIX
		MULTI_VALUE MODULE_LIST
		REQUIRED TARGET PREFIX MODULE_LIST
		ARGN ${ARGN})

	foreach(module ${MODULE_LIST})

		foreach(property MAIN STACK PRIORITY)
			get_target_property(${property} ${module} ${property})
			if(NOT ${property})
				set(${property} ${${property}_DEFAULT})
			endif()
		endforeach()

		if (MAIN)
			set(ln_name "${PREFIX}${MAIN}")
			add_custom_command(TARGET ${TARGET}
				POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E create_symlink ${TARGET} ${ln_name}
				WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
			)
		endif()
	endforeach()
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
		-D__TC_POSIX
		-Dnoreturn_function=__attribute__\(\(noreturn\)\)
		)

	include_directories(platforms/posix/include)

	if ("${TC_BOARD}" MATCHES "sitl")

		if(UNIX AND APPLE)
			add_definitions(-D__TC_DARWIN)

		elseif(CYGWIN)
			add_definitions(
				-D__TC_CYGWIN
				-D_GNU_SOURCE
				-D__USE_LINUX_IOCTL_DEFS
				-U__CUSTOM_FILE_IO__
				)
		else()
			add_definitions(-D__TC_LINUX)
		endif()

	endif()

	add_compile_options($<$<COMPILE_LANGUAGE:C>:-Wbad-function-cast>)

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
	target_link_libraries(prebuild_targets INTERFACE tc_layer drivers_board)
	add_dependencies(prebuild_targets DEPENDS uorb_headers)

endfunction()
