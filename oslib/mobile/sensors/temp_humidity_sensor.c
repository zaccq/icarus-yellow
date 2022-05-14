/*
**********************************************************************************
* @file oslib/scu_drivers/scu_sensor/temp_humidity_sensor.c
* @author Matthew Kumar - 45303494
* @date 1-04-2022
* @brief Temperature and Humidity values functionalitiy
**********************************************************************************
* EXTERNAL FUNCTIONS
**********************************************************************************
* int hts221_get_temp_humidity_sen(char* type, char* buf) - gets the temp/hum
* values for procesing
**********************************************************************************
*/

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <sys/util.h>
#include <string.h>
#include <logging/log.h>

#include "temp_humidity_sensor.h"

LOG_MODULE_REGISTER(hts221_log_module, LOG_LEVEL_DBG);


struct sensor_value temp, hum;

/**
 * @brief Function reads the temperature and stores it in struct 
 * 
 * @param dev device of what we are trying to use
 * @param buf the buffer where we are to write the receieved values to
 */
void process_temp_sample(const struct device *dev, char* buf) {

	if (sensor_sample_fetch(dev) < 0) {
		printf("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
		printf("Cannot read HTS221 temperature channel\n");
		return;
	}
	
	sprintf(buf, "%.1f", sensor_value_to_double(&temp));
	
}


/**
 * @brief Function getting the humidity values from the sensor
 * 
 * @param dev device of what we are trying to use
 * @param buf the buffer where we are to write the receieved values to
 */
void process_hum_sample(const struct device *dev, char* buf) {

	if (sensor_sample_fetch(dev) < 0) {
		printf("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum) < 0) {
		printf("Cannot read HTS221 humidity channel\n");
		return;
	}

	sprintf(buf, "%.1f", sensor_value_to_double(&hum));
}

/**
 * @brief Call back function for the hts221_handler
 * 
 * @param dev device of what we are trying to use
 * @param trig provides the sensor trigger
 */
void hts221_handler(const struct device *dev,
			   const struct sensor_trigger *trig) {
}

/**
 * @brief Function gets the temp/hum values for procesing
 * 
 * @param type is the type of data we want 
 * @param buf provides the array for the data to be loaded into
 */
int hts221_get_temp_humidity_sen(char* type, char* buf) {
	const struct device *dev = device_get_binding("HTS221");

	if (dev == NULL) {
		LOG_ERR("Could not get HTS221 device\n");
		return 1;
	}

	if (IS_ENABLED(CONFIG_HTS221_TRIGGER)) {
		struct sensor_trigger trig = {
			.type = SENSOR_TRIG_DATA_READY,
			.chan = SENSOR_CHAN_ALL,
		};
		if (sensor_trigger_set(dev, &trig, hts221_handler) < 0) {
			LOG_ERR("Cannot configure trigger\n");
			return 1;
		}
	}

	if (strcmp(type, "temp") == 0) {

		process_temp_sample(dev, buf);

	} else if (strcmp(type, "hum") == 0) {

		process_hum_sample(dev, buf);
	}
	
	return 0;
}