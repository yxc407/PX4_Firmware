
find_program(JAVA_ANT_PATH "ant")
find_package(Java)

if(JAVA_ANT_PATH AND Java_JAVAC_EXECUTABLE AND Java_JAVA_EXECUTABLE)

	tc_add_git_submodule(TARGET git_jmavsim PATH "${TC_SOURCE_DIR}/Tools/simulation/jmavsim/jMAVSim")

	add_custom_target(jmavsim_run_symlink
		COMMAND ${CMAKE_COMMAND} -E create_symlink ${TC_SOURCE_DIR}/Tools/simulation/jmavsim/jmavsim_run.sh ${TC_BINARY_DIR}/rootfs/jmavsim_run.sh
		BYPRODUCTS ${TC_BINARY_DIR}/rootfs/jmavsim_run.sh
	)

	# build_jmavsim
	add_custom_command(
		OUTPUT ${TC_SOURCE_DIR}/Tools/simulation/jmavsim/jMAVSim/out/production/jmavsim_run.jar
		COMMAND ${JAVA_ANT_PATH} create_run_jar copy_res
		WORKING_DIRECTORY ${TC_SOURCE_DIR}/Tools/simulation/jmavsim/jMAVSim/
		USES_TERMINAL
		DEPENDS git_jmavsim jmavsim_run_symlink
		COMMENT "building jMAVSim"
	)
	add_custom_target(build_jmavsim
		DEPENDS
			${TC_SOURCE_DIR}/Tools/simulation/jmavsim/jMAVSim/out/production/jmavsim_run.jar
			jmavsim_run_symlink
	)

	# launch helper
	add_custom_target(jmavsim_iris
		COMMAND ${CMAKE_COMMAND} -E env TC_SYS_AUTOSTART=10017 $<TARGET_FILE:tc>
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
		USES_TERMINAL
		DEPENDS
			tc git_jmavsim build_jmavsim jmavsim_run_symlink
			${TC_SOURCE_DIR}/ROMFS/tcfmu_common/init.d-posix/airframes/10017_jmavsim_iris
		COMMENT "launching tc jmavsim_iris (SYS_AUTOSTART=10017)"
	)
	add_custom_target(jmavsim DEPENDS jmavsim_iris) # alias

endif()
