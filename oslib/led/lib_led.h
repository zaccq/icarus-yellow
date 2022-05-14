 /** 
 **************************************************************
 * @file lib_led.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief Led library Header file
 ***************************************************************
 */

#ifndef LIB_LED_H
#define LIB_LED_H

#include <stdint.h>
#include <drivers/gpio.h>

#define LED0_GREEN 0
#define LED1_RED 1
#define LED1_GREEN 2
#define LED1_BLUE 3

/* Function Prototypes..*/
int8_t gpio_config(int8_t led_index, gpio_flags_t extra_flags);
int8_t led_on(int8_t led_index);
int8_t led_set(int8_t led_index, int status);
int8_t led_off(int8_t led_index);
int8_t led_toggle(int8_t led_index);
int8_t led_dc(int8_t led_index);
int8_t led_init(int8_t led_index);
int8_t led_deinit(int8_t led_index);

#endif // LIB_LED_H
