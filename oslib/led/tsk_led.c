 /** 
 **************************************************************
 * @file tsk_led.c
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief Led task Source file
 ***************************************************************
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include "lib_led.h"
#include "tsk_led.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(tsk_led_log_module, LOG_LEVEL_DBG);

K_THREAD_STACK_DEFINE(led_ctrl_stack_area, LED_CTRL_STACK_SIZE);
struct k_thread led_ctrl_thread_data;
void tsk_led_ctrl_entry_point(void *a, void *b, void *c);

K_MSGQ_DEFINE(led_ctrl_msgq, sizeof(led_ctrl_packet_t), 10, 4);

k_tid_t led_ctrl_tsk_init() {

    return k_thread_create(&led_ctrl_thread_data, led_ctrl_stack_area,
        K_THREAD_STACK_SIZEOF(led_ctrl_stack_area), tsk_led_ctrl_entry_point,
        NULL, NULL, NULL, LED_CTRL_THREAD_PRIORITY, 0, K_NO_WAIT);
    
    k_thread_name_set(&led_ctrl_thread_data, "AHU LED CTRL");

}

void tsk_led_ctrl_entry_point(void *a, void *b, void *c) {

    led_init(LED0_GREEN);
    led_init(LED1_RED);
    led_init(LED1_GREEN);
    led_init(LED1_BLUE);
    led_ctrl_packet_t data;

    while(1) {
        if (k_msgq_get(&led_ctrl_msgq, &data, K_FOREVER) == 0) {
            LOG_DBG("Led control message received: <%d, %d, %d>", data.led, data.action, data.status);
            switch(data.action) {
                case init_led:
                    led_init((uint8_t)(data.led));
                    break;
                case deinit_led:
                    led_deinit((uint8_t)(data.led));
                    break;
                case set_led:
                    led_set((uint8_t)(data.led), data.status);
                    break;
                case toggle_led:
                    led_toggle((uint8_t)(data.led));
                    break;
                default:
                    break;
            }
        }
    }

}

