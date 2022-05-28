#ifndef MPU9250_WRAPPERS_H
#define MPU9250_WRAPPERS_H

struct mpu9250_sens_cfg {
    uint8_t feat_mask;
    uint16_t gyro_fsr;
    uint8_t accel_fsr;
    uint8_t lpf_cfreq;
    uint8_t sample_rate;
};

struct mpu9250_dmp_cfg {
    uint16_t feat_mask;
    uint8_t sample_rate;
};

struct mpu9250_reading {
    int8_t accel_xyz[3];
    int8_t gyro_xyz[3];
    int8_t magn_xyz[3];
    int8_t quat_wxyz[4];
}

int mobile_mpu9250_init(struct mpu9250_sens_cfg* sensor_cfg,
        struct mpu9250_dmp_cfg* dmp_cfg);
uint8_t fifo_available();
int8_t dmp_update_fifo();
void compute_euler_angles(uint8_t degrees);
void print_imu_data();

#endif //MPU9250_WRAPPERS_H