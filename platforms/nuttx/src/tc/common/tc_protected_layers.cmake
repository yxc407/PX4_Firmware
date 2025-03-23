
# Build the user side tc_layer

add_library(tc_layer
	board_dma_alloc.c
	board_fat_dma_alloc.c
	tasks.cpp
	console_buffer_usr.cpp
	cdc_acm_check.cpp
	${TC_SOURCE_DIR}/platforms/posix/src/tc/common/print_load.cpp
	${TC_SOURCE_DIR}/platforms/posix/src/tc/common/cpuload.cpp
	tc_userspace_init.cpp
	tc_usr_crypto.cpp
	tc_mtd.cpp
	usr_board_ctrl.c
	usr_hrt.cpp
	usr_mcu_version.cpp
)

target_link_libraries(tc_layer
	PRIVATE
		m
		nuttx_c
		nuttx_xx
		nuttx_mm
)

# Build the interface library between user and kernel side
add_library(tc_board_ctrl
	board_ctrl.c
	board_ioctl.c
	hrt_ioctl.c
)

add_dependencies(tc_board_ctrl nuttx_context tc_kernel_builtin_list_target)
target_compile_options(tc_board_ctrl PRIVATE -D__KERNEL__)

target_link_libraries(tc_layer
	PUBLIC
		board_bus_info
)

# Build the kernel side tc_kernel_layer

add_library(tc_kernel_layer
	${KERNEL_SRCS}
)

target_link_libraries(tc_kernel_layer
	PRIVATE
		${KERNEL_LIBS}
		nuttx_kc
		nuttx_karch
		nuttx_kmm
)

target_link_libraries(tc_kernel_layer
	PUBLIC
		board_bus_info
)

if (DEFINED TC_CRYPTO)
	target_link_libraries(tc_kernel_layer PUBLIC crypto_backend)
	target_link_libraries(tc_layer PUBLIC crypto_backend_interface)
endif()

add_dependencies(tc_kernel_layer prebuild_targets)
target_compile_options(tc_kernel_layer PRIVATE -D__KERNEL__)
target_link_libraries(tc_kernel_layer PUBLIC tc_board_ctrl)
