/* temp_humidity_sensor.h
 * Created: 19/03/2021
 * Updated:
 * Author: Matthew Kumar
 */

#ifndef TEMP_HUMIDITY_SENSOR_H
#define TEMP_HUMIDITY_SENSOR_H

extern struct sensor_value temp;
extern struct sensor_value hum;

int hts221_get_temp_humidity_sen(char* type, char* buf);


#endif

