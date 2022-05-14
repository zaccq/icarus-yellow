 /** 
 **************************************************************
 * @file lib_led.c
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief Led library Source file
 ***************************************************************
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include "lib_led.h"
#include <logging/log.h>

/* Device Tree Macros */
#define LED0_G_NODE DT_NODELABEL(led0_green)
#define LED1_R_NODE DT_NODELABEL(led1_red)
#define LED1_G_NODE DT_NODELABEL(led1_green)
#define LED1_B_NODE DT_NODELABEL(led1_blue)

#define LED_INDEX_VALID 1
#define LED_INDEX_INVALID 0

LOG_MODULE_REGISTER(lib_led_log_module, LOG_LEVEL_DBG);

static const struct gpio_dt_spec led[4] =
    {GPIO_DT_SPEC_GET(LED0_G_NODE, gpios),
    GPIO_DT_SPEC_GET(LED1_R_NODE, gpios),
    GPIO_DT_SPEC_GET(LED1_G_NODE, gpios),
    GPIO_DT_SPEC_GET(LED1_B_NODE, gpios)};

static inline uint8_t valid_led_index(int8_t led_index) {

    if(led_index < 0 || led_index > 3) {
        return LED_INDEX_INVALID;
    }
    return LED_INDEX_VALID;

}

int8_t gpio_config(int8_t led_index, gpio_flags_t extra_flags) {

    if(valid_led_index(led_index) == LED_INDEX_INVALID) {
        return -1;
    }
    return gpio_pin_configure_dt(&(led[led_index]), extra_flags);

}

int8_t led_on(int8_t led_index) {

    LOG_DBG("Led %d turned on", led_index);
    return gpio_config(led_index, GPIO_OUTPUT_ACTIVE);
    
}

int8_t led_set(int8_t led_index, int status) {

    if(valid_led_index(led_index) == LED_INDEX_INVALID) {
        return -1;
    }
    return gpio_pin_set_dt(&led[led_index], status);
    
}

int8_t led_off(int8_t led_index) {

    LOG_DBG("Led %d turned off", led_index);
    return gpio_config(led_index, GPIO_OUTPUT_INACTIVE);
    
}

int8_t led_toggle(int8_t led_index) {

    LOG_DBG("Led %d toggled", led_index);
    return gpio_pin_toggle_dt(&(led[led_index]));
    
}

int8_t led_dc(int8_t led_index) {

    LOG_DBG("Led %d disconnected", led_index);
    return gpio_config(led_index, GPIO_DISCONNECTED);

}

int8_t led_init(int8_t led_index) {

    LOG_DBG("Led %d initialised", led_index);
    if(valid_led_index(led_index) == LED_INDEX_INVALID) {
        return -1;
    }

    if (!device_is_ready((led[led_index]).port)) {
		return 1;
	}

	if (led_off(led_index) >= 0) {
		return 0;
	}

	return 1;
}

int8_t led_deinit(int8_t led_index) {

    LOG_DBG("Led %d deinitialised", led_index);
	if (led_dc(led_index) >= 0) {
		return 0;
	}

	return 1;

}