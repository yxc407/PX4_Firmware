
include_directories(${CMAKE_CURRENT_BINARY_DIR})

get_property(module_libraries GLOBAL PROPERTY TC_MODULE_LIBRARIES)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

set(TC_SHELL_COMMAND_PREFIX "tc-")

add_definitions("-DTC_SHELL_COMMAND_PREFIX=\"${TC_SHELL_COMMAND_PREFIX}\"")

tc_posix_generate_builtin_commands(
	OUT apps
	MODULE_LIST ${module_libraries})

tc_posix_generate_alias(
	OUT ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/tc-alias.sh
	MODULE_LIST ${module_libraries}
	PREFIX ${TC_SHELL_COMMAND_PREFIX}
)

if (CMAKE_BUILD_TYPE STREQUAL FuzzTesting)
	add_executable(tc
		src/tc/common/main_fuzztesting.cpp
		apps.cpp
		)
else()
	add_definitions(-DTC_SOURCE_DIR="${TC_SOURCE_DIR}" -DTC_BINARY_DIR="${TC_BINARY_DIR}")

	add_executable(tc
		src/tc/common/main.cpp
		apps.cpp
		)
endif()

target_link_libraries(tc
	PRIVATE
		${module_libraries}
		m
		parameters
)

if((NOT APPLE) AND (NOT ANDROID))
	target_link_libraries(tc PRIVATE rt)
endif()

if(NOT ANDROID)
	target_link_libraries(tc PRIVATE pthread)
endif()

target_link_libraries(tc PRIVATE uORB)

if (CMAKE_BUILD_TYPE STREQUAL FuzzTesting)
	target_include_directories(tc PRIVATE SYSTEM "${CMAKE_BINARY_DIR}/mavlink}")
	target_compile_options(tc PRIVATE "-Wno-cast-align")
endif()

#=============================================================================
# install
#

# TODO: extend to snapdragon

# tc dirs
install(
	DIRECTORY
		${PROJECT_SOURCE_DIR}/posix-configs
		${PROJECT_SOURCE_DIR}/test
		${CMAKE_BINARY_DIR}/etc
		${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
	DESTINATION
		${PROJECT_NAME}
	USE_SOURCE_PERMISSIONS
)

# Module Symlinks
tc_posix_generate_symlinks(
	MODULE_LIST ${module_libraries}
	PREFIX ${TC_SHELL_COMMAND_PREFIX}
	TARGET tc
)

if(config_romfs_root)
	add_subdirectory(${TC_SOURCE_DIR}/ROMFS ${TC_BINARY_DIR}/ROMFS)
	add_dependencies(tc romfs_gen_files_target)
endif()


# board defined upload helper
if(EXISTS "${TC_BOARD_DIR}/cmake/upload.cmake")
	include(${TC_BOARD_DIR}/cmake/upload.cmake)
endif()

# board defined link libraries
if(EXISTS "${TC_BOARD_DIR}/cmake/link_libraries.cmake")
	include(${TC_BOARD_DIR}/cmake/link_libraries.cmake)
endif()

if("${TC_BOARD}" MATCHES "beaglebone_blue")
	target_link_libraries(tc PRIVATE robotics_cape)

elseif("${TC_BOARD}" MATCHES "emlid_navio2")
	target_link_libraries(tc PRIVATE atomic)

	# vscode launch.json
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Debug/launch_rpi.json.in ${TC_SOURCE_DIR}/.vscode/launch.json COPYONLY)

elseif("${TC_BOARD}" MATCHES "sitl")

	# vscode launch.json
	if(${TC_BOARD_LABEL} MATCHES "replay")
		configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Debug/launch_replay.json.in ${TC_SOURCE_DIR}/.vscode/launch.json COPYONLY)

	else()
		configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Debug/launch_sitl.json.in ${TC_SOURCE_DIR}/.vscode/launch.json COPYONLY)
	endif()

	set(SITL_WORKING_DIR ${TC_BINARY_DIR}/rootfs)
	file(MAKE_DIRECTORY ${SITL_WORKING_DIR})
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Debug/posix.lldbinit ${SITL_WORKING_DIR}/.lldbinit COPYONLY)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Debug/posix.gdbinit ${SITL_WORKING_DIR}/.gdbinit COPYONLY)

	if(BUILD_TESTING)
		include(sitl_tests)
	endif()

	# "none" legacy SITL helper target
	add_custom_target(none
		COMMAND $<TARGET_FILE:tc>
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
		USES_TERMINAL
		DEPENDS tc
	)

	# install

	# tc dirs
	install(
		DIRECTORY
			${PROJECT_SOURCE_DIR}/integrationtests
			${PROJECT_SOURCE_DIR}/launch
			${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
		DESTINATION
			${PROJECT_NAME}
		USE_SOURCE_PERMISSIONS
		)

	# tc files
	install(
		FILES
			${PROJECT_SOURCE_DIR}/CMakeLists.txt
			${PROJECT_SOURCE_DIR}/package.xml
		DESTINATION
			${PROJECT_NAME}
		)

	# tc Tools dirs
	install(
		DIRECTORY
			${PROJECT_SOURCE_DIR}/Tools/ecl_ekf
		DESTINATION
			${PROJECT_NAME}/Tools
		USE_SOURCE_PERMISSIONS
		)

	# tc Tools files
	install(
		PROGRAMS
			${PROJECT_SOURCE_DIR}/Tools/simulation/gazebo-classic/setup_gazebo.bash
			${PROJECT_SOURCE_DIR}/Tools/upload_log.py
		DESTINATION
			${PROJECT_NAME}/Tools
		)

	# ROMFS files
	install(
		DIRECTORY
			${TC_BINARY_DIR}/etc
		DESTINATION
			${PROJECT_NAME}/build/tc_sitl_default
		)

	# gazebo dirs
	install(
		DIRECTORY
			${PROJECT_SOURCE_DIR}/Tools/simulation/gz/models
			${PROJECT_SOURCE_DIR}/Tools/simulation/gz/worlds
		DESTINATION
			${PROJECT_NAME}/Tools/simulation/gazebo-classic/sitl_gazebo-classic
		)

endif()
