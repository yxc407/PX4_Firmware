############################################################################
#
#   Copyright (c) 2019 TC Development Team. All rights reserved.
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

find_program(JLinkGDBServerCLExe_PATH JLinkGDBServerCLExe
	HINTS /Applications/SEGGER/JLink
)
if(JLinkGDBServerCLExe_PATH)
	# jlink_upload (flash binary)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Debug/jlink_gdb_start.sh.in ${TC_BINARY_DIR}/jlink_gdb_start.sh @ONLY)
	add_custom_target(jlink_upload
		COMMAND ${TC_BINARY_DIR}/jlink_gdb_start.sh
		COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/Debug/upload_jlink_gdb.sh $<TARGET_FILE:tc>
		DEPENDS
			tc
			${TC_BINARY_DIR}/jlink_gdb_start.sh
			${CMAKE_CURRENT_SOURCE_DIR}/Debug/upload_jlink_gdb.sh
		WORKING_DIRECTORY ${TC_BINARY_DIR}
		USES_TERMINAL
	)

	# jlink_gdb_backtrace (attach, print current tasks, back trace, exit)
	add_custom_target(jlink_gdb_backtrace
		COMMAND ${TC_BINARY_DIR}/jlink_gdb_start.sh
		COMMAND ${CMAKE_COMMAND} -E env WORKSPACE=${TC_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/Debug/jlink_gdb_backtrace.sh $<TARGET_FILE:tc>
		DEPENDS
			tc
			${TC_BINARY_DIR}/jlink_gdb_start.sh
			${CMAKE_CURRENT_SOURCE_DIR}/Debug/jlink_gdb_backtrace.sh
		WORKING_DIRECTORY ${TC_BINARY_DIR}
		USES_TERMINAL
	)

	# jlink_gdb_backtrace_simple (attach, print current tasks, back trace, exit)
	add_custom_target(jlink_gdb_backtrace_simple
		COMMAND ${TC_BINARY_DIR}/jlink_gdb_start.sh
		COMMAND ${CMAKE_COMMAND} -E env WORKSPACE=${TC_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/Debug/jlink_gdb_backtrace_simple.sh $<TARGET_FILE:tc>
		DEPENDS
			tc
			${TC_BINARY_DIR}/jlink_gdb_start.sh
			${CMAKE_CURRENT_SOURCE_DIR}/Debug/jlink_gdb_backtrace_simple.sh
		WORKING_DIRECTORY ${TC_BINARY_DIR}
		USES_TERMINAL
	)


	# jlink_upload_bootloader
	#   board directory supplied bootloader.bin
	if(TARGET bootloader_elf)
		# jlink_upload_bootloader
		add_custom_target(jlink_upload_bootloader
			COMMAND ${TC_BINARY_DIR}/jlink_gdb_start.sh
			COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/Debug/upload_jlink_gdb.sh ${TC_BINARY_DIR}/${TC_BOARD_VENDOR}_${TC_BOARD_MODEL}_bootloader.elf
			DEPENDS
				${TC_BINARY_DIR}/${TC_BOARD_VENDOR}_${TC_BOARD_MODEL}_bootloader.elf
				${TC_BINARY_DIR}/jlink_gdb_start.sh
				${CMAKE_CURRENT_SOURCE_DIR}/Debug/upload_jlink_gdb.sh
			WORKING_DIRECTORY ${TC_BINARY_DIR}
			USES_TERMINAL
		)
	endif()

endif()

# jlink_debug_gdb (flash binary and run with gdb attached)
find_program(JLinkGDBServerExe_PATH JLinkGDBServerExe
	HINTS /Applications/SEGGER/JLink
)
if(JLinkGDBServerExe_PATH AND CMAKE_GDB)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Debug/jlink_debug_gdb.sh.in ${TC_BINARY_DIR}/jlink_debug_gdb.sh @ONLY)
	add_custom_target(jlink_debug_gdb
		COMMAND ${TC_BINARY_DIR}/jlink_debug_gdb.sh
		DEPENDS
			tc
			${TC_BINARY_DIR}/.gdbinit
			${TC_BINARY_DIR}/jlink_debug_gdb.sh
		WORKING_DIRECTORY ${TC_BINARY_DIR}
		USES_TERMINAL
	)
endif()

# jlink_debug_ozone (run Segger Ozone debugger with current target configuration)
find_program(Ozone_PATH Ozone ozone
	HINTS /Applications/Ozone.app/Contents/MacOS/
)
if(Ozone_PATH)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Debug/jlink_debug_ozone.sh.in ${TC_BINARY_DIR}/jlink_debug_ozone.sh @ONLY)
	add_custom_target(jlink_debug_ozone
		COMMAND ${TC_BINARY_DIR}/jlink_debug_ozone.sh
		DEPENDS
			tc
			${TC_BINARY_DIR}/jlink_debug_ozone.sh
		WORKING_DIRECTORY ${TC_BINARY_DIR}
		USES_TERMINAL
	)
endif()

# .bin flashing
find_program(JLinkExe_PATH JLinkExe)
if(JLinkExe_PATH)

	# jlink_flash_bootloader_bin
	if(EXISTS ${TC_BOARD_DIR}/extras/${TC_BOARD_VENDOR}_${TC_BOARD_MODEL}_bootloader.bin)

		set(BOARD_FIRMWARE_BIN "${TC_BOARD_VENDOR}_${TC_BOARD_MODEL}_bootloader.bin")
		set(BOARD_FIRMWARE_APP_OFFSET "0x08000000")
		configure_file(${TC_SOURCE_DIR}/platforms/nuttx/Debug/flash_bin.jlink.in ${TC_BINARY_DIR}/flash_bootloader_bin.jlink @ONLY)

		add_custom_target(jlink_flash_bootloader_bin
			${CMAKE_COMMAND} -E copy_if_different ${TC_BOARD_DIR}/extras/${TC_BOARD_VENDOR}_${TC_BOARD_MODEL}_bootloader.bin ${TC_BINARY_DIR}/${BOARD_FIRMWARE_BIN}
			COMMAND ${JLinkExe_PATH} -CommandFile ${TC_BINARY_DIR}/flash_bootloader_bin.jlink
			DEPENDS
				${TC_SOURCE_DIR}/platforms/nuttx/Debug/flash_bin.jlink.in
			WORKING_DIRECTORY ${TC_BINARY_DIR}
			USES_TERMINAL
		)

	endif()

	# jlink_flash_bin
	if(uavcan_bl_image_name)
		# uavcan signed firmware
		set(BOARD_FIRMWARE_BIN ${uavcan_bl_image_name})
		set(BOARD_FIRMWARE_APP_OFFSET "0x08010000")
		configure_file(${TC_SOURCE_DIR}/platforms/nuttx/Debug/flash_bin.jlink.in ${TC_BINARY_DIR}/flash_bin.jlink @ONLY)

		add_custom_target(jlink_flash_bin
			COMMAND ${JLinkExe_PATH} -CommandFile ${TC_BINARY_DIR}/flash_bin.jlink
			DEPENDS
				${TC_SOURCE_DIR}/platforms/nuttx/Debug/flash_bin.jlink.in
				${TC_BINARY_DIR}/${uavcan_bl_image_name}
			WORKING_DIRECTORY ${TC_BINARY_DIR}
			USES_TERMINAL
		)
	else()
		# regular firmware ${TC_BINARY_DIR}/${TC_CONFIG}.bin
		set(BOARD_FIRMWARE_BIN ${TC_CONFIG}.bin)
		set(BOARD_FIRMWARE_APP_OFFSET "0x08008000") # TODO: get from board
		configure_file(${TC_SOURCE_DIR}/platforms/nuttx/Debug/flash_bin.jlink.in ${TC_BINARY_DIR}/flash_bin.jlink @ONLY)

		add_custom_target(jlink_flash_bin
			COMMAND ${CMAKE_COMMAND} -E echo "WARNING jlink_flash_bin currently assumes starting address ${BOARD_FIRMWARE_APP_OFFSET}"
			COMMAND ${JLinkExe_PATH} -CommandFile ${TC_BINARY_DIR}/flash_bin.jlink
			DEPENDS
				${TC_SOURCE_DIR}/platforms/nuttx/Debug/flash_bin.jlink.in
				${TC_BINARY_DIR}/${TC_CONFIG}.bin
			WORKING_DIRECTORY ${TC_BINARY_DIR}
			USES_TERMINAL
		)
	endif()

endif()
