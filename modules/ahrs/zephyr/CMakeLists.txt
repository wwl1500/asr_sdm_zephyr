# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include)
zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include/fusion)

zephyr_library_sources_ifdef(CONFIG_ASR_AHRS ${ZEPHYR_CURRENT_MODULE_DIR}/src/ahrs.c)
zephyr_library_sources_ifdef(CONFIG_ASR_AHRS ${ZEPHYR_CURRENT_MODULE_DIR}/src/FusionAhrs.c)
zephyr_library_sources_ifdef(CONFIG_ASR_AHRS ${ZEPHYR_CURRENT_MODULE_DIR}/src/fusion_offset.c)
zephyr_library_sources_ifdef(CONFIG_ASR_AHRS ${ZEPHYR_CURRENT_MODULE_DIR}/src/FusionCompass.c)
zephyr_library_sources_ifdef(CONFIG_ASR_AHRS_THREAD ${ZEPHYR_CURRENT_MODULE_DIR}/src/ahrs_thread.c)
