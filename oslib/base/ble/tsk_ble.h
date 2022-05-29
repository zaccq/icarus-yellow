 /** 
 **************************************************************
 * @file tsk_ble.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief Bluetooth task header file
 ***************************************************************
 */

#ifndef TSK_BLE_H
#define TSK_BLE_H

#include <stdint.h>
#include "zephyr.h"

#define BLE_MSG_MAX_PAYLOAD_LEN 16

#define BLE_CTRL_THREAD_PRIORITY 4
#define BLE_CTRL_THREAD_STACK_SIZE 2048

#define BLE_LED_THREAD_PRIORITY 6
#define BLE_LED_THREAD_STACK_SIZE 512

k_tid_t ble_ctrl_tsk_init(void);
k_tid_t ble_led_tsk_init(void);

typedef enum {
    ble_none = 0,
    ble_hts221_temp = 1,
    ble_hts221_humidity = 2,
    ble_lps22_pressure = 3,
    ble_ccs811_volatiles = 4,
    ble_lis2dh_x = 5,
    ble_lis2dh_y = 6,
    ble_lis2dh_z = 7,
    ble_rgb_led = 8,
    ble_buzzer = 9,
    ble_pushbutton = 10,
    ble_dc = 11,
    ble_sample = 12,
    ble_all = 13,
    ble_ccs811_co2 = 14,
    ble_rssi = 15,
    ble_ultra = 16,
    ble_mpu9250 = 17
} ble_ctrl_dest_t;

typedef enum {
    ble_request=0x01,
    ble_response=0x02
} ble_packet_type_t;

typedef struct ble_ctrl_request_packet {
    ble_ctrl_dest_t dest;
    uint8_t payload_length;
    char payload[BLE_MSG_MAX_PAYLOAD_LEN+1];
} ble_ctrl_request_packet_t;

int8_t ble_ctrl_request_send(ble_ctrl_request_packet_t* msg);

#endif //TSK_BLE_H