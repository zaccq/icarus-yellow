/*
***************************************************************************
* @file oslib/scu_drivers/scu_io/scu_led.c
* @author Matthew Kumar - 45303494
* @date 1-04-2021
* @brief LED functionalitiy
***************************************************************************
* EXTERNAL FUNCTIONS
***************************************************************************
* void get_led() - control LED
***************************************************************************
*/

#include <zephyr.h>
#include <drivers/gpio.h>
#include <string.h>
#include <logging/log.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0", "led1", "led2" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)


// Init GPIO pins for the three LEDS
static const struct gpio_dt_spec ledR = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec ledG = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec ledB = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

// Flag Variables for checking mode of LED
int toggleRed = 0;
int toggleGreen = 0;
int toggleBlue = 0;

/**
 * @brief Red LED is initalised and has 
 * 3 different functionality based on status
 * 
 * @param status controls the functionality
 */
void led_red(char status) {
	int ret;

	if (!device_is_ready(ledR.port)) {
		return;
	}

	ret = gpio_pin_configure_dt(&ledR, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	if (status == 'o') {
		toggleRed = 1;
		ret = gpio_pin_set_dt(&ledR, toggleRed);
		if (ret < 0) {
			return;
		}
	} else if (status == 'f') {
		toggleRed = 0;
		ret = gpio_pin_set_dt(&ledR, toggleRed);
		if (ret < 0) {
			return;
		}

	} else if (status == 't') {
		ret = gpio_pin_set_dt(&ledR, !toggleRed);
		toggleRed = !toggleRed;
		if (ret < 0) {
			return;
		}
	}
}


/**
 * @brief Green LED is initalised and has 
 * 3 different functionality based on status
 * 
 * @param status controls the functionality
 */
void led_green(char status) {
	int ret;

	if (!device_is_ready(ledG.port)) {
		return;
	}

	ret = gpio_pin_configure_dt(&ledG, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	if (status == 'o') {
		toggleGreen = 1;
		ret = gpio_pin_set_dt(&ledG, toggleGreen);
		if (ret < 0) {
			return;
		}
	} else if (status == 'f') {
		toggleGreen = 0;
		ret = gpio_pin_set_dt(&ledG, toggleGreen);
		if (ret < 0) {
			return;
		}

	} else if (status == 't') {
		ret = gpio_pin_set_dt(&ledG, !toggleGreen);
		toggleGreen = !toggleGreen;
		if (ret < 0) {
			return;
		}
	}
}

/**
 * @brief Blue LED is initalised and has 
 * 3 different functionality based on status
 * 
 * @param status controls the functionality
 */
void led_blue(char status) {
	int ret;

	if (!device_is_ready(ledB.port)) {
		return;
	}

	ret = gpio_pin_configure_dt(&ledB, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	if (status == 'o') {
		toggleBlue = 1;
		ret = gpio_pin_set_dt(&ledB, toggleBlue);
		if (ret < 0) {
			return;
		}
	} else if (status == 'f') {
		toggleBlue = 0;
		ret = gpio_pin_set_dt(&ledB, toggleBlue);
		if (ret < 0) {
			return;
		}

	} else if (status == 't') {
		ret = gpio_pin_set_dt(&ledB, !toggleBlue);
		toggleBlue = !toggleBlue;
		if (ret < 0) {
			return;
		}
	}
}

/**
 * @brief Function changes the colours and 
 * mode of the leds based on AHU input
 * 
 * @param status controls the functionality
 */
void get_led(char* payload) {

	char status;

    if (strcmp(payload,"r1") == 0) {
		status = 'o';
		led_red(status);

    } else if (strcmp(payload,"r0") == 0) {
		status = 'f';
		led_red(status);

	} else if (strcmp(payload, "r2") == 0) {
		status = 't';
		led_red(status);

	} else if (strcmp(payload,"g1") == 0) {
		status = 'o';
		led_green(status);

	} else if (strcmp(payload,"g0") == 0) {
		status = 'f';
		led_green(status);

	} else if (strcmp(payload,"g2") == 0) {
		status = 't';
		led_green(status);
	} else if (strcmp(payload,"b1") == 0) {
		status = 'o';
		led_blue(status);
	} else if (strcmp(payload,"b0") == 0) {
		status = 'f';
		led_blue(status);

	} else if (strcmp(payload,"b2") == 0) {
		status = 't';
		led_blue(status);
	}
}