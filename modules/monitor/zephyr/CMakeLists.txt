# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include)

zephyr_library_sources_ifdef(CONFIG_ASR_CPU_MONITOR
	${ZEPHYR_CURRENT_MODULE_DIR}/src/cpu_monitor.c
	${ZEPHYR_CURRENT_MODULE_DIR}/src/cpu_monitor_thread.c
)
