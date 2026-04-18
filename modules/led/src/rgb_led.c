/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <asr/rgb_led.h>

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(asr_rgb_led, LOG_LEVEL_INF);

#define STRIP_NODE      DT_ALIAS(led_strip)
#define STRIP_NUM_LEDS  DT_PROP(STRIP_NODE, chain_length)

static const struct device *const strip_dev = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgb pixels[STRIP_NUM_LEDS];

int asr_rgb_led_init(void)
{
	if (!device_is_ready(strip_dev)) {
		LOG_ERR("LED strip device not ready");
		return -ENODEV;
	}

	memset(pixels, 0, sizeof(pixels));
	int ret = led_strip_update_rgb(strip_dev, pixels, STRIP_NUM_LEDS);

	if (ret == 0) {
		LOG_INF("RGB LED initialised");
	}
	return ret;
}

int asr_rgb_led_set(uint8_t r, uint8_t g, uint8_t b)
{
	pixels[0].r = r;
	pixels[0].g = g;
	pixels[0].b = b;
	return led_strip_update_rgb(strip_dev, pixels, STRIP_NUM_LEDS);
}

int asr_rgb_led_off(void)
{
	return asr_rgb_led_set(0, 0, 0);
}
