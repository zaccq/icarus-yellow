#ifndef MPU9250_WRAPPERS_H
#define MPU9250_WRAPPERS_H

int mobile_mpu9250_init(uint8_t sensors, uint8_t dmp_features, uint8_t dmp_sample_rate);
uint8_t fifo_available();
int8_t dmp_update_fifo();
void compute_euler_angles(uint8_t degrees);
void print_imu_data();

#endif //MPU9250_WRAPPERS_H