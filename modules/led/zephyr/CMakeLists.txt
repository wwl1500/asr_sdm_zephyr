# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include)

zephyr_library_sources_ifdef(CONFIG_ASR_USR_LED ${ZEPHYR_CURRENT_MODULE_DIR}/src/usr_led.c)
zephyr_library_sources_ifdef(CONFIG_ASR_RGB_LED ${ZEPHYR_CURRENT_MODULE_DIR}/src/rgb_led.c)
zephyr_library_sources_ifdef(CONFIG_ASR_LED_THREAD ${ZEPHYR_CURRENT_MODULE_DIR}/src/led_thread.c)
