############################################################################
#
# Copyright (c) 2017 TC Development Team. All rights reserved.
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
# 	utility functions
#
#		* tc_generate_airframes_xml
#

#=============================================================================
#
#	tc_generate_airframes_xml
#
#	Generates airframes.xml
#
#	Usage:
#		tc_generate_airframes_xml(OUT <airframe-xml-file>)
#
#	Input:
#		XML : the airframes.xml file
#		BOARD : the board
#
#	Output:
#		OUT	: the generated source files
#
#	Example:
#		tc_generate_airframes_xml(OUT airframes.xml)
#
function(tc_generate_airframes_xml)
	tc_parse_function_args(
		NAME tc_generate_airframes_xml
		ONE_VALUE BOARD
		REQUIRED BOARD
		ARGN ${ARGN})

	add_custom_command(OUTPUT ${TC_BINARY_DIR}/airframes.xml
		COMMAND ${PYTHON_EXECUTABLE} ${TC_SOURCE_DIR}/Tools/px_process_airframes.py
			--airframes-path ${TC_SOURCE_DIR}/ROMFS/${config_romfs_root}/init.d
			--board CONFIG_ARCH_BOARD_${TC_BOARD}
			--xml ${TC_BINARY_DIR}/airframes.xml
		DEPENDS ${TC_SOURCE_DIR}/Tools/px_process_airframes.py
		COMMENT "Creating airframes.xml"
		)
	add_custom_target(airframes_xml DEPENDS ${TC_BINARY_DIR}/airframes.xml)
endfunction()
