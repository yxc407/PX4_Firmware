cmake_minimum_required(VERSION 3.9 FATAL_ERROR)

set(TC_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE FILEPATH "TC source directory" FORCE)
set(TC_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE FILEPATH "TC binary directory" FORCE)

list(APPEND CMAKE_MODULE_PATH ${TC_SOURCE_DIR}/cmake)
include(tc_parse_function_args)

#=============================================================================
# git
#
include(tc_git)

execute_process(
	COMMAND git describe --exclude ext/* --always --tags
	OUTPUT_VARIABLE TC_GIT_TAG
	OUTPUT_STRIP_TRAILING_WHITESPACE
	WORKING_DIRECTORY ${TC_SOURCE_DIR}
	)

# git describe to X.Y.Z version
string(REPLACE "." ";" VERSION_LIST ${TC_GIT_TAG})

# major version
list(GET VERSION_LIST 0 TC_VERSION_MAJOR)
string(REPLACE "v" "" TC_VERSION_MAJOR ${TC_VERSION_MAJOR})
# minor version
list(GET VERSION_LIST 1 TC_VERSION_MINOR)
# patch version
list(GET VERSION_LIST 2 TC_VERSION_PATCH)
string(REPLACE "-" ";" TC_VERSION_PATCH ${TC_VERSION_PATCH})
list(GET TC_VERSION_PATCH 0 TC_VERSION_PATCH)

message(STATUS "TC version: ${TC_GIT_TAG} (${TC_VERSION_MAJOR}.${TC_VERSION_MINOR}.${TC_VERSION_PATCH})")

define_property(GLOBAL PROPERTY TC_MODULE_LIBRARIES
                 BRIEF_DOCS "TC module libs"
                 FULL_DOCS "List of all TC module libraries"
                 )

define_property(GLOBAL PROPERTY TC_KERNEL_MODULE_LIBRARIES
                 BRIEF_DOCS "TC kernel side module libs"
                 FULL_DOCS "List of all TC kernel module libraries"
                 )

define_property(GLOBAL PROPERTY TC_MODULE_PATHS
                 BRIEF_DOCS "TC module paths"
                 FULL_DOCS "List of paths to all TC modules"
                 )
define_property(GLOBAL PROPERTY TC_SRC_FILES
                 BRIEF_DOCS "src files from all TC modules & libs"
                 FULL_DOCS "SRC files from tc_add_{module,library}"
                 )

#=============================================================================
# configuration
#

include(tc_add_module) # in cmake/tc_add_module.cmake
set(config_module_list)
set(config_kernel_list)

# Find Python
find_package(PythonInterp 3)
# We have a custom error message to tell users how to install python3.
if(NOT PYTHONINTERP_FOUND)
	message(FATAL_ERROR "Python 3 not found. Please install Python 3:\n"
		"    Ubuntu: sudo apt install python3 python3-dev python3-pip\n"
		"    macOS: brew install python")
endif()

option(PYTHON_COVERAGE "Python code coverage" OFF)
if(PYTHON_COVERAGE)
	message(STATUS "python coverage enabled")
	set(PYTHON_EXECUTABLE coverage run -p)
endif()

include(tc_config) # in cmake/tc_config.cmake, TC_BOARD is define here
include(kconfig)
message(STATUS "TC config: ${TC_CONFIG}")
message(STATUS "TC platform: ${TC_PLATFORM}")

if($ENV{CLION_IDE})
	# CLion automatically executes some compiler commands after configuring the
	# project. This would fail on NuttX, as visibility.h tries to (indirectly)
	# include nuttx/config.h, which at that point does not exist yet
	add_definitions(-DTC_DISABLE_GCC_POISON)
endif()

if(${TC_PLATFORM} STREQUAL "posix")
	if(ENABLE_LOCKSTEP_SCHEDULER)
		add_definitions(-DENABLE_LOCKSTEP_SCHEDULER)
		message(STATUS "TC lockstep: enabled")
	else()
		message(STATUS "TC lockstep: disabled")
	endif()
endif()

# external modules
set(EXTERNAL_MODULES_LOCATION "" CACHE STRING "External modules source location")

if(NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	get_filename_component(EXTERNAL_MODULES_LOCATION "${EXTERNAL_MODULES_LOCATION}" ABSOLUTE)
endif()

set_property(GLOBAL PROPERTY TC_MODULE_CONFIG_FILES)

include(platforms/${TC_PLATFORM}/cmake/tc_impl_os.cmake)
list(APPEND CMAKE_MODULE_PATH ${TC_SOURCE_DIR}/platforms/${TC_PLATFORM}/cmake)

if(EXISTS "${TC_SOURCE_DIR}/platforms/${TC_PLATFORM}/cmake/init.cmake")
	include(init)
endif()

#=============================================================================
# project definition
#
project(tc CXX C ASM)

# CMake build type (Debug Release RelWithDebInfo MinSizeRel Coverage)
if(NOT CMAKE_BUILD_TYPE)
	if(${TC_PLATFORM} STREQUAL "nuttx")
		set(TC_BUILD_TYPE "MinSizeRel")
	else()
		set(TC_BUILD_TYPE "RelWithDebInfo")
	endif()

	set(CMAKE_BUILD_TYPE ${TC_BUILD_TYPE} CACHE STRING "Build type" FORCE)
endif()

if((CMAKE_BUILD_TYPE STREQUAL "Debug") OR (CMAKE_BUILD_TYPE STREQUAL "Coverage"))
	set(MAX_CUSTOM_OPT_LEVEL -O0)
elseif(CMAKE_BUILD_TYPE MATCHES "Sanitizer")
	set(MAX_CUSTOM_OPT_LEVEL -O1)
elseif(CMAKE_BUILD_TYPE MATCHES "Release")
	set(MAX_CUSTOM_OPT_LEVEL -O3)
else()
	if(tc_constrained_flash_build)
		set(MAX_CUSTOM_OPT_LEVEL -Os)
	else()
		set(MAX_CUSTOM_OPT_LEVEL -O2)
	endif()
endif()

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel;Coverage;AddressSanitizer;UndefinedBehaviorSanitizer")
message(STATUS "cmake build type: ${CMAKE_BUILD_TYPE}")

# Check if LTO option and check if toolchain supports it
if(LTO)
    include(CheckIPOSupported)
    check_ipo_supported()
    message(AUTHOR_WARNING "LTO enabled: LTO is highly experimental and should not be used in production")
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

set(package-contact "tcusers@googlegroups.com")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${TC_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${TC_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${TC_BINARY_DIR})

#=============================================================================

# gold linker - use if available (posix only for now)
if(${TC_PLATFORM} STREQUAL "posix")
	include(CMakeDependentOption)
	CMAKE_DEPENDENT_OPTION(USE_LD_GOLD
			"Use GNU gold linker" ON
			"NOT WIN32;NOT APPLE" OFF
	)

	if(USE_LD_GOLD)
		execute_process(COMMAND ${CMAKE_C_COMPILER} -fuse-ld=gold -Wl,--version ERROR_QUIET OUTPUT_VARIABLE LD_VERSION)
		if("${LD_VERSION}" MATCHES "GNU gold")
			set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=gold")
			set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fuse-ld=gold")
		else()
			set(USE_LD_GOLD OFF)
		endif()
	endif()
endif()

#=============================================================================

# Setup install paths
if(${TC_PLATFORM} STREQUAL "posix")
	# This makes it possible to dynamically load code which depends on symbols
	# inside the tc executable.
	set(CMAKE_POSITION_INDEPENDENT_CODE ON)
	set(CMAKE_ENABLE_EXPORTS ON)

	if(CMAKE_BUILD_TYPE MATCHES "Coverage")
		include(coverage)
	endif()

	include(sanitizers)

	# Define GNU standard installation directories
	include(GNUInstallDirs)

	if (NOT CMAKE_INSTALL_PREFIX)
		set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "Install path prefix" FORCE)
	endif()
endif()

include(ccache)

#=============================================================================
# get chip and chip manufacturer
#
tc_os_determine_build_chip() #in platforms/nuttx/cmake/tc_impl_os.cmake
if(NOT TC_CHIP_MANUFACTURER)
	message(FATAL_ERROR "tc_os_determine_build_chip() needs to set TC_CHIP_MANUFACTURER")
endif()
if(NOT TC_CHIP)
	message(FATAL_ERROR "tc_os_determine_build_chip() needs to set TC_CHIP")
endif()

#=============================================================================
# build flags
#
include(tc_add_common_flags)
tc_add_common_flags() #in cmake/tc_add_common_flags.cmake
tc_os_add_flags() #in platforms/nuttx/cmake/tc_impl_os.cmake

#=============================================================================
# board cmake init (optional)
#
if(EXISTS ${TC_BOARD_DIR}/cmake/init.cmake)
	include(${TC_BOARD_DIR}/cmake/init.cmake)
endif()

#=============================================================================
# message, and airframe generation
#
include(tc_metadata)

add_subdirectory(msg EXCLUDE_FROM_ALL)

tc_generate_airframes_xml(BOARD ${TC_BOARD}) #in cmake/metadata.cmake

#=============================================================================
# external projects
#
set(ep_base ${TC_BINARY_DIR}/external)
set_property(DIRECTORY PROPERTY EP_BASE ${ep_base})

# add external project install folders to build
# add the directories so cmake won't warn
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${ep_base})
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${ep_base}/Install)
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${ep_base}/Install/lib)
link_directories(${ep_base}/Install/lib)
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${ep_base}/Install/include)
include_directories(${ep_base}/Install/include)

#=============================================================================
# external modules
#
set(external_module_paths)
if (NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	message(STATUS "External modules: ${EXTERNAL_MODULES_LOCATION}")
	add_subdirectory("${EXTERNAL_MODULES_LOCATION}/src" external_modules)

	foreach(external_module ${config_module_list_external})
		add_subdirectory(${EXTERNAL_MODULES_LOCATION}/src/${external_module} external_modules/${external_module})
		list(APPEND external_module_paths ${EXTERNAL_MODULES_LOCATION}/src/${external_module})
	endforeach()
endif()

#=============================================================================
# Testing - Automatic unit and integration testing with CTest
#

# optionally enable cmake testing (supported only on posix)
option(CMAKE_TESTING "Configure test targets" OFF)
if(${TC_CONFIG} STREQUAL "tc_sitl_test")
	set(CMAKE_TESTING ON)
endif()
if(CMAKE_TESTING)
	include(CTest) # sets BUILD_TESTING variable
endif()

# enable test filtering to run only specific tests with the ctest -R regex functionality
set(TESTFILTER "" CACHE STRING "Filter string for ctest to selectively only run specific tests (ctest -R)")

# if testing is enabled download and configure gtest
list(APPEND CMAKE_MODULE_PATH ${TC_SOURCE_DIR}/cmake/gtest/)
include(tc_add_gtest)
if(BUILD_TESTING)
	include(gtest)

	add_custom_target(test_results
			COMMAND GTEST_COLOR=1 ${CMAKE_CTEST_COMMAND} --output-on-failure -T Test -R ${TESTFILTER} USES_TERMINAL
			DEPENDS
				tc
				examples__dyn_hello
			USES_TERMINAL
			COMMENT "Running tests"
			WORKING_DIRECTORY ${TC_BINARY_DIR})
	set_target_properties(test_results PROPERTIES EXCLUDE_FROM_ALL TRUE)
endif()


#=============================================================================
# subdirectories
#
add_library(parameters_interface INTERFACE)
add_library(kernel_parameters_interface INTERFACE)

include(tc_add_library)
add_subdirectory(src/lib EXCLUDE_FROM_ALL)

add_subdirectory(platforms/${TC_PLATFORM}/src/tc)
add_subdirectory(platforms EXCLUDE_FROM_ALL)

if(EXISTS "${TC_BOARD_DIR}/CMakeLists.txt")
	add_subdirectory(${TC_BOARD_DIR})
endif()

foreach(module ${config_module_list})
	add_subdirectory(src/${module})
endforeach()

# add events lib after modules and libs as it needs to know all source files (TC_SRC_FILES)
add_subdirectory(src/lib/events EXCLUDE_FROM_ALL)
# metadata needs TC_MODULE_CONFIG_FILES
add_subdirectory(src/lib/metadata EXCLUDE_FROM_ALL)

# must be the last module before firmware
add_subdirectory(src/lib/parameters EXCLUDE_FROM_ALL)

if(${TC_PLATFORM} STREQUAL "nuttx" AND NOT CONFIG_BUILD_FLAT)
	target_link_libraries(parameters_interface INTERFACE usr_parameters)
	target_link_libraries(kernel_parameters_interface INTERFACE parameters)
else()
	target_link_libraries(parameters_interface INTERFACE parameters)
endif()

# firmware added last to generate the builtin for included modules
add_subdirectory(platforms/${TC_PLATFORM})

#=============================================================================
# uORB graph generation: add a custom target 'uorb_graph'
#
set(uorb_graph_config ${TC_BOARD})

set(graph_module_list "")
foreach(module ${config_module_list})
	set(graph_module_list "${graph_module_list}" "--src-path" "src/${module}")
endforeach()

add_custom_command(OUTPUT ${uorb_graph_config}
	COMMAND ${PYTHON_EXECUTABLE} ${TC_SOURCE_DIR}/Tools/uorb_graph/create.py
		${graph_module_list} --src-path src/lib
		--merge-depends
		--exclude-path src/examples
		--exclude-path src/lib/parameters # FIXME: enable & fix
		--file ${TC_SOURCE_DIR}/Tools/uorb_graph/graph_${uorb_graph_config}
	WORKING_DIRECTORY ${TC_SOURCE_DIR}
	COMMENT "Generating uORB graph"
)
add_custom_target(uorb_graph DEPENDS ${uorb_graph_config})


include(bloaty)


include(doxygen)
include(metadata)
include(package)

# install python requirements using configured python
add_custom_target(install_python_requirements
	COMMAND ${PYTHON_EXECUTABLE} -m pip install --requirement ${TC_SOURCE_DIR}/Tools/setup/requirements.txt
	DEPENDS ${TC_SOURCE_DIR}/Tools/setup/requirements.txt
	USES_TERMINAL
)

if(EXISTS "${TC_SOURCE_DIR}/platforms/${TC_PLATFORM}/cmake/finalize.cmake")
	include(finalize)
endif()
