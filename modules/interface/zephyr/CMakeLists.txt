# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include)

zephyr_library_sources_ifdef(CONFIG_ASR_USB_INTERFACE
	${ZEPHYR_CURRENT_MODULE_DIR}/src/usb_protocol.c
	${ZEPHYR_CURRENT_MODULE_DIR}/src/usb_thread.c
)
