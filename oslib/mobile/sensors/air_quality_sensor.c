/*
***************************************************************************
* @file oslib/scu_drivers/scu_sensors/air_quality_sensor.c
* @author Matthew Kumar - 45303494
* @date 1-04-2022
* @brief VOC reading functionalitiy
***************************************************************************
* EXTERNAL FUNCTIONS
***************************************************************************
* void get_air_quality_val(char* buf) - Function gets sensor values
***************************************************************************
*/

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <sys/printk.h>
#include <drivers/sensor/ccs811.h>
#include <stdio.h>

static bool app_fw_2;

static int do_fetch(const struct device *dev, char* buf) {
	struct sensor_value co2, tvoc, voltage, current;
	int rc = 0;

	if (rc == 0) {
		rc = sensor_sample_fetch(dev);
	}
	if (rc == 0) {
		const struct ccs811_result_type *rp = ccs811_result(dev);

		sensor_channel_get(dev, SENSOR_CHAN_CO2, &co2);
		sensor_channel_get(dev, SENSOR_CHAN_VOC, &tvoc);
		sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &voltage);
		sensor_channel_get(dev, SENSOR_CHAN_CURRENT, &current);

		sprintf(buf, "%u", tvoc.val1);
		// WIll need to add another function for CO2 if you choose to implement it


		if (app_fw_2 && !(rp->status & CCS811_STATUS_DATA_READY)) {
			printk("STALE DATA\n");
		}

		if (rp->status & CCS811_STATUS_ERROR) {
			printk("ERROR: %02x\n", rp->error);
		}
	}
	return rc;
}

/**
 * @brief Function gets the VOC reading
 * 
 * @param buf stores the value in this 
 * variable for SCU to packet in main
 */
void get_air_quality_val(char* buf) {
	const struct device *dev = device_get_binding(DT_LABEL(DT_INST(0, ams_ccs811)));
	struct ccs811_configver_type cfgver;
	int rc;

	if (!dev) {
		printk("Failed to get device binding");
		return;
	}

	rc = ccs811_configver_fetch(dev, &cfgver);

	if (rc == 0) {
		do_fetch(dev, buf);
	}
}
