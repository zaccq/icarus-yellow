/*
**********************************************************************************
* @file oslib/scu_drivers/scu_sensor/temp_pressure_sensor.c
* @author Matthew Kumar - 45303494
* @date 1-04-2022
* @brief Temperature and Pressure values functionalitiy
**********************************************************************************
* EXTERNAL FUNCTIONS
**********************************************************************************
* int lps22hb_get_temp_pres_values(char* type, char* buf)- gets temp and 
* pressure values for processing
**********************************************************************************
*/

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <sys/util.h>
#include <string.h>
#include <logging/log.h>

#include "temp_pressure_sensor.h"

struct sensor_value pressure;
struct sensor_value lps22hbTemp;

LOG_MODULE_REGISTER(lps22hb_log_module, LOG_LEVEL_DBG);

/**
 * @brief Function ensures that sensor is working before
 * extracting values of pressure and temperature
 * 
 * @param dev the device we are trying to get values from
 */
void process_sample_temp_pres(const struct device *dev) {
	static unsigned int obs;

	if (sensor_sample_fetch(dev) < 0) {
		printf("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure) < 0) {
		printf("Cannot read LPS22HB pressure channel\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &lps22hbTemp) < 0) {
		printf("Cannot read LPS22HB temperature channel\n");
		return;
	}

	++obs;
	printf("Observation:%u\n", obs);

	/* display pressure */
	printf("Pressure:%.1f kPa\n", sensor_value_to_double(&pressure));

	/* display temperature */
	printf("Temperature:%.1f C\n", sensor_value_to_double(&lps22hbTemp));

}

/**
 * @brief Function processes the temperature values
 * 
 * @param buf gives a buffer for the read values to be processed into
 * @param dev the device we are trying to get values from
 */
void process_temp_value(const struct device *dev, char* buf) {

	if (sensor_sample_fetch(dev) < 0) {
		printf("Sensor sample update error\n");
		return;
	}


	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &lps22hbTemp) < 0) {
		printf("Cannot read LPS22HB temperature channel\n");
		return;
	}

	// Set the temp value to the buffer
	sprintf(buf, "%.1f", sensor_value_to_double(&lps22hbTemp));

}

/**
 * @brief Function processes the Pressure values
 * 
 * @param buf gives a buffer for the read values to be processed into
 * @param dev the device we are trying to get values from
 */
void process_pressure_value(const struct device *dev, char* buf) {

	if (sensor_sample_fetch(dev) < 0) {
		printf("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure) < 0) {
		printf("Cannot read LPS22HB pressure channel\n");
		return;
	}

	// Set the pressure value ot the buffer
	sprintf(buf, "%.1f", sensor_value_to_double(&pressure));

}

/**
 * Function gets the temperature and pressure values
 */
int lps22hb_get_temp_pres_values(char* type, char* buf) {
	LOG_DBG("start of the pressure temp area");
	const struct device *dev = device_get_binding(DT_LABEL(DT_INST(0, st_lps22hb_press)));
	
	if (dev == NULL) {
		LOG_ERR("Could not get LPS22HB device\n");
		return 1;
	}

	if (strcmp(type, "temp") == 0) {

			process_temp_value(dev, buf);

	} else if (strcmp(type, "pres") == 0) {
			process_pressure_value(dev, buf);
	}

	return 0;

}
