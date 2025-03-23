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


add_subdirectory(tc_daemon)
add_subdirectory(lockstep_scheduler)

set(EXTRA_DEPENDS)

add_library(tc_layer
	tc_posix_impl.cpp
	tasks.cpp
	tc_sem.cpp
	tc_init.cpp
	lib_crc32.c
	drv_hrt.cpp
	cpuload.cpp
	print_load.cpp
)
target_compile_definitions(tc_layer PRIVATE MODULE_NAME="tc")
target_compile_options(tc_layer PRIVATE -Wno-cast-align) # TODO: fix and enable
target_link_libraries(tc_layer PRIVATE work_queue tc_work_queue)
target_link_libraries(tc_layer PRIVATE tc_daemon drivers_board)

if(ENABLE_LOCKSTEP_SCHEDULER)
	target_link_libraries(tc_layer PRIVATE lockstep_scheduler)
endif()


if(EXTRA_DEPENDS)
	add_dependencies(tc_layer ${EXTRA_DEPENDS})
endif()


if(BUILD_TESTING)
	add_subdirectory(test_stubs)
	add_subdirectory(gtest_runner)
endif()
