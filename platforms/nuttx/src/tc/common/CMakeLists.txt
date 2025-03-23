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

# skip for tc_layer support on an IO board
if(NOT TC_BOARD MATCHES "io-v2")
	# Kernel side & nuttx flat build common sources
	set(KERNEL_SRCS
		board_crashdump.c
		board_dma_alloc.c
		board_fat_dma_alloc.c
		cdc_acm_check.cpp
		console_buffer.cpp
		cpuload.cpp
		gpio.c
		print_load.cpp
		tasks.cpp
		tc_nuttx_impl.cpp
		tc_init.cpp
		tc_manifest.cpp
		tc_mtd.cpp
		tc_24xxxx_mtd.c
		tc_crypto.cpp
		progmem_dump.c
	)

	# Kernel side & nuttx flat build common libraries
	set(KERNEL_LIBS
		arch_board_reset
		arch_board_critmon
		arch_version
		nuttx_sched
	)

if (NOT DEFINED CONFIG_BUILD_FLAT AND "${TC_PLATFORM}" MATCHES "nuttx")
	# Build the NuttX user and kernel space tc layers
	include(${CMAKE_CURRENT_SOURCE_DIR}/tc_protected_layers.cmake)

else()
	# Build the flat build tc_layer
	include(${CMAKE_CURRENT_SOURCE_DIR}/tc_layer.cmake)
endif()

else()
	# Build the tc layer for io_v2
	add_library(tc_layer ${TC_SOURCE_DIR}/platforms/common/empty.c)
endif()

add_dependencies(tc_layer prebuild_targets)

add_subdirectory(gpio)
add_subdirectory(srgbled)

# Build tc_random
if (DEFINED TC_CRYPTO)
	add_library(tc_random nuttx_random.c)
	target_link_libraries(tc_random PRIVATE nuttx_crypto)
endif()
