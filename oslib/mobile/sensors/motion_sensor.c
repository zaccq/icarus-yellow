/*
**********************************************************************************
* @file oslib/scu_drivers/scu_sensor/motion_sensor.c
* @author Matthew Kumar - 45303494
* @date 1-04-2022
* @brief accelerometer values functionalitiy
**********************************************************************************
* EXTERNAL FUNCTIONS
**********************************************************************************
* void get_motion_values_lis2dh(char type, char* buf) - gets values for processing
**********************************************************************************
*/

#include <stdio.h>
#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <string.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(motion_sensor_log_module, LOG_LEVEL_DBG);

/**
 * @brief Function gets the accelerometer 
 * 
 * @param freq the frequency the buzzer is set at
 */
static void fetch_and_display(const struct device *sensor, char type,
		 char* buf) {

	
	static unsigned int count;
	struct sensor_value accel[3];
	//struct sensor_value temperature;
	const char *overrun = "";
	int rc = sensor_sample_fetch(sensor);

	++count;
	if (rc == -EBADMSG) {
		/* Sample overrun.  Ignore in polled mode. */
		if (IS_ENABLED(CONFIG_LIS2DH_TRIGGER)) {
			overrun = "[OVERRUN] ";
		}
		rc = 0;
	}
	if (rc == 0) {
		rc = sensor_channel_get(sensor,
					SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc < 0) {
		printf("ERROR: Update failed: %d\n", rc);
	} else {
		if (type == 'x') {
			sprintf(buf, "%.1f", sensor_value_to_double(&accel[0]));
			LOG_DBG("Do we get here? - after sprintf @ %s", buf);

		} else if (type == 'y') {
			sprintf(buf, "%.1f", sensor_value_to_double(&accel[1]));
		} else if (type == 'z') {
			sprintf(buf, "%.1f", sensor_value_to_double(&accel[2]));
		}
	}
}

/**
 * @brief Function gets the accelerometer 
 * values and sends to scu for processsing
 * 
 * @param type is type of value (x,y,z)
 * @param buf is where we store the reading once we get it
 */
void get_motion_values_lis2dh(char type, char* buf) {

	const struct device *sensor = DEVICE_DT_GET_ANY(st_lis2dh);

	if (sensor == NULL) {
		printf("No device found\n");
		return;
	}
	if (!device_is_ready(sensor)) {
		printf("Device %s is not ready\n", sensor->name);
		return;
	}

	fetch_and_display(sensor, type, buf);

}
