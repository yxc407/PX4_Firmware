############################################################################
#
#   Copyright (c) 2015 TC Development Team. All rights reserved.
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

set(SRCS)

if(NOT "${TC_PLATFORM}" MATCHES "qurt" AND NOT "${TC_BOARD}" MATCHES "io-v2" AND NOT "${TC_BOARD_LABEL}" MATCHES "bootloader")
	list(APPEND SRCS
		tc_log.cpp
		tc_log_history.cpp
	)
endif()

add_library(tc_platform STATIC
	board_common.c
	board_identity.c
	events.cpp
	external_reset_lockout.cpp
	i2c.cpp
	i2c_spi_buses.cpp
	module.cpp
	tc_getopt.c
	tc_cli.cpp
	shutdown.cpp
	spi.cpp
	${SRCS}
)
target_link_libraries(tc_platform prebuild_targets tc_work_queue)

if(NOT "${TC_BOARD}" MATCHES "io-v2")
	add_subdirectory(uORB)
endif()

add_subdirectory(tc_work_queue)
add_subdirectory(work_queue)

if("${TC_PLATFORM}" MATCHES "nuttx")
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/c_cpp_properties.json.nuttx.in ${TC_SOURCE_DIR}/.vscode/c_cpp_properties.json @ONLY)
else()
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/c_cpp_properties.json.in ${TC_SOURCE_DIR}/.vscode/c_cpp_properties.json @ONLY)
endif()

tc_add_unit_gtest(SRC board_identity_test.cpp LINKLIBS tc_platform)
