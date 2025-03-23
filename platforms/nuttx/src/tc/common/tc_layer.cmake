# Build the tc layer for nuttx flat build

add_library(tc_layer
		${KERNEL_SRCS}
		cdc_acm_check.cpp
	)

target_link_libraries(tc_layer
	PRIVATE
		${KERNEL_LIBS}
		nuttx_c
		nuttx_arch
		nuttx_mm
	)


if (DEFINED TC_CRYPTO)
	target_link_libraries(tc_layer
		PUBLIC
			crypto_backend
	)
endif()

target_link_libraries(tc_layer PRIVATE tc_platform)
