# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include)

zephyr_library_sources_ifdef(CONFIG_ASR_LED ${ZEPHYR_CURRENT_MODULE_DIR}/src/led.c)
zephyr_library_sources_ifdef(CONFIG_ASR_PWM_LED_THREAD ${ZEPHYR_CURRENT_MODULE_DIR}/src/pwm_led.c)
