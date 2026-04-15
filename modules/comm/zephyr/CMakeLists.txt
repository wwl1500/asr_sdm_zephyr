# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include)

zephyr_library_sources_ifdef(CONFIG_ASR_COMM
	${ZEPHYR_CURRENT_MODULE_DIR}/src/comm_thread.c
	${ZEPHYR_CURRENT_MODULE_DIR}/src/protocol.c
)
