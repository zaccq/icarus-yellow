/* scu_ble.h
 * Created: 25/03/2022
 * Updated:
 * Author: Matthew Kumar
 */

#ifndef SCU_BLE_H
#define SCU_BLE_H

struct packet {
    uint8_t preamble;
	uint8_t type;
	uint8_t dest;
	uint8_t len;
	char payload[19]; // Arbitary value but will change later
};

extern void scu_ble_init(void);

extern struct k_msgq ahu_msg_to_scu;
extern struct k_msgq scu_msg_to_ahu;

#endif