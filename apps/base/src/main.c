/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */


//#include <logging/log.h>

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>
#include "tsk_led.h"
#include "tsk_ble.h"
#include <logging/log.h>
#include "lib_shell.h"


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

LOG_MODULE_REGISTER(main_log_module, LOG_LEVEL_DBG);

void main(void) {

	/* Setup DTR */
    uint32_t dtr = 0;

    /* Enable the USB Driver */
    if (usb_enable(NULL))   
        return;
    
   /* Wait on DTR - 'Data Terminal Ready'
    * Will wait here until a terminal has been attached to the device
    * This is not necessary, however, can be useful for printing boot info etc..
    */
    while (!dtr) {
        uart_line_ctrl_get(shell_dev, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(50));
    }
    
    LOG_DBG("Shell initialised successfully!");

    k_tid_t led_ctrl_thread_id = led_ctrl_tsk_init();
    LOG_DBG("LED control thread started successfully!");

    k_tid_t ble_ctrl_thread_id = ble_ctrl_tsk_init();
    LOG_DBG("BLE control thread started successfully!");

    k_tid_t ble_led_ctrl_thread_id = ble_led_tsk_init();
    LOG_DBG("BLE status LED control thread started successfully!");

    return;
	
}