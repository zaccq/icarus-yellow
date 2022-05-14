 /** 
 **************************************************************
 * @file task_led.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief Led task Header file
 ***************************************************************
 */

#ifndef TSK_LED_H
#define TSK_LED_H

#include <stdint.h>
#include "zephyr.h"

#define LED_CTRL_THREAD_PRIORITY 5
#define LED_CTRL_STACK_SIZE 500

k_tid_t led_ctrl_tsk_init(void);

typedef enum {init_led, deinit_led, set_led, toggle_led} led_ctrl_action_t;
typedef enum {led0_g, led1_r, led1_g, led1_b} led_t;

extern struct k_msgq led_ctrl_msgq;

typedef struct led_ctrl_packet {
    led_t led;
    led_ctrl_action_t action;
    uint8_t status;
} led_ctrl_packet_t;

#endif //TSK_LED_H