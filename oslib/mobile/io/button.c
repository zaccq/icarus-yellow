/*
***************************************************************
* @file oslib/scu_io/button.c
* @author Matthew Kumar - 45303494
* @date 1-04-2021
* @brief Functionality for the button
***************************************************************
*/

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>
#include <drivers/regulator.h>
#include <drivers/pwm.h>
#include <string.h>

#define SLEEP_TIME_MS	1
#define SW0_NODE	DT_ALIAS(sw0)
#define SPK_PWR_NODE DT_NODELABEL(spk_pwr)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button_cb_data;

/**
 * @brief Function is a call back to confirm button is pressed
 * 
 * @param dev is the device you checking
 * @param cb the call back pointer
 * @param pins
 */
void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins) {
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

/**
 * @brief Function enables the button
 * 
 * @param buf variable sets the status of button
 */
void enable_button(char* buf) {
	int ret;

	if (!device_is_ready(button.port)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	int val = gpio_pin_get_dt(&button);

	if (val == 1) {
		buf[0] = '1';
	} else {
		buf[0] = '0';
	}
	
}
