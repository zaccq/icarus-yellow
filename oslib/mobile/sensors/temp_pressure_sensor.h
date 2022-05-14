/* temp_humidity_sensor.h
 * Created: 19/03/2021
 * Updated:
 * Author: Matthew Kumar
 */

#ifndef TEMP_PRESSURE_SENSOR_H
#define TEMP_PRESSURE_SENSOR_H

extern struct sensor_value lps22hbTemp;
extern struct sensor_value pressure;

int lps22hb_get_temp_pres_values(char* type, char* buf);

#endif