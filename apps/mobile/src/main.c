/*
***************************************************************
* @file pracs/prac1/scu/main.c
* @author Matthew Kumar - 45303494
* @date 1-04-2022
* @brief main function for prac 1
***************************************************************
*/
#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <sys/util.h>
#include <string.h>
#include <stdlib.h>
#include <logging/log.h>
#include <drivers/regulator.h>

#include "temp_humidity_sensor.h"
#include "temp_pressure_sensor.h"
#include "motion_sensor.h"
#include "air_quality_sensor.h"
#include "scu_ble.h"
#include "button.h"
#include "led.h"
#include "buzzer.h"

#include "tsk_mpu9250.h"

// Define stack size and priority for threading
#define STACK_SIZE 1024
#define PRIORITY 3

// Initialise Packet variables
struct packet scuMessage;
// Init glaobal variables 
uint8_t allMode = 0;
uint8_t sampleTime = 10;
uint8_t dutyCycle = 100;
uint16_t freq = 0;

void main() {

    k_tid_t mpu9250_tid = mpu9250_tsk_init();

    while(1) {
        k_msleep(1000);
    }

}

/**
 * @brief Function gets the sensor values from peripherals and enables button
 * Function is used in threading entry function
 * 
 * @param t variable gets the type that ahu is requesting
 */
void get_value(uint8_t t) {

	// Init char array for received info from sensors
    char data[19]; // was 19
    memset(data, 0x0, 19);

	// Packet Formatting
	scuMessage.preamble = 0xAA;
	scuMessage.type = 0x02; 

	switch(t) {
		case 1:
			hts221_get_temp_humidity_sen("temp", data);
			break;
			
		case 2: 
			hts221_get_temp_humidity_sen("hum", data);
			break;
		case 3:
			lps22hb_get_temp_pres_values("pres", data);
			break;
		case 4: 
			get_air_quality_val(data);
			break;
		case 5:
			get_motion_values_lis2dh('x', data);
			break;
		case 6:
			get_motion_values_lis2dh('y', data);
			break;
		case 7:
			get_motion_values_lis2dh('z', data);
			break;
		case 8:
			get_led(scuMessage.payload);
			break;
		case 10:
			enable_button(data);
            break;
        case 14:
			enable_button(data);
	}
	
	// Start to load the packet info to be sent back to AHU
	scuMessage.dest = t;
    strcpy(scuMessage.payload, data);
	scuMessage.len = strlen(data) + 1;
	
	// Send the packet to debug
    if (k_msgq_put(&scu_msg_to_ahu, &scuMessage, K_NO_WAIT) != 0) {

        k_msgq_purge(&scu_msg_to_ahu);
    }


}


/**
 * @brief Helper function to get all sensor values
 */
void all() {
	for(uint8_t i = 1; i <= 7; i++) {
		get_value(i);
		k_msleep(1);
	}
}

/**
 * @brief Helper function to get all sensor values
 * 
 * @param a void pointer
 * @param b void pointer
 * @param c void pointer
 */
void all_thread(void *a, void *b, void *c) {

	while(1) {

		if(allMode == 1) {
			all();
		}

		k_sleep(K_SECONDS(sampleTime));
	}

}

/**
 * @brief Threading function for the SCU. Entry function allows
 * the scu to process the packets that are sent from the AHU
 * 
 * @param a void pointer
 * @param b void pointer
 * @param c void pointer
 */
void scu_thread_function(void *a, void *b, void *c) {

	struct packet data;
	struct buzzer_freq freqVal;

	//buzzer_init(); // Initalise the buzzer & PWM related stuff

	while(1) {

		if (k_msgq_get(&ahu_msg_to_scu, &data, K_FOREVER) == 0) {
			if (data.preamble == 0xAA) {

				if (data.dest == 13) {
					if(data.payload[0] == '1') {
						allMode = 1;
					} else {
						allMode = 0;
					}
				} else if (data.dest == 12) {
					char* endPtr;
					sampleTime = strtol(data.payload, &endPtr, 10);

				} else if (data.dest == 11) {
					char* endPtr;
					dutyCycle = strtol(data.payload, &endPtr, 10);

				} else if (data.dest == 9) {
					freqVal.freq = atoi(data.payload);

					/*if (k_msgq_put(&buzzer_queue, &freqVal, K_NO_WAIT) != 0) {

						k_msgq_purge(&buzzer_queue); // If queue is full
					}*/
					
				} else if (data.dest == 8) {
					get_led(data.payload);

				} else {
					get_value(data.dest);
				}
			}
		}
	}	
}

// Define the threads used in this function
K_THREAD_DEFINE(scu_value_id, STACK_SIZE, scu_thread_function,
				 NULL, NULL, NULL, PRIORITY, 0, 0); 

K_THREAD_DEFINE(all_value_id, STACK_SIZE, all_thread,
				 NULL, NULL, NULL, PRIORITY, 0, 0); 