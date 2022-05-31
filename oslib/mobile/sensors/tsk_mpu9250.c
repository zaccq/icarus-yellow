#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <toolchain.h>
#include <logging/log.h>
#include <drivers/i2c.h>

#include <sys/printk.h>
#include "lib_mpu9250.h"

#include "tsk_mpu9250.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu9250_registers.h"
#include "scu_ble.h"


#define MPU9250_THREAD_PRIORITY 4
#define MPU9250_THREAD_STACK_SIZE 2048

#define MPU9250_LPF_CFREQ 20
#define MPU9250_SAMPLE_RATE 40
#define MPU9250_DMP_SAMPLE_RATE 40

K_THREAD_STACK_DEFINE(mpu9250_thread_stack_area, MPU9250_THREAD_STACK_SIZE);
struct k_thread mpu9250_thread_data;

struct device* mpu9250_i2c_dev = NULL;

void tsk_mpu9250_entry_point(void *a, void *b, void *c);

k_tid_t mpu9250_tsk_init(void) {

    return k_thread_create(&mpu9250_thread_data, mpu9250_thread_stack_area,
        K_THREAD_STACK_SIZEOF(mpu9250_thread_stack_area),
        tsk_mpu9250_entry_point, NULL, NULL, NULL, MPU9250_THREAD_PRIORITY,
        0, K_MSEC(10));

}

void tsk_mpu9250_entry_point(void *a, void *b, void *c) {

    struct mpu9250_sens_cfg sensor_cfg = {0};

    sensor_cfg.feat_mask = INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS;
    sensor_cfg.gyro_fsr = 250;
    sensor_cfg.accel_fsr = 2;
    sensor_cfg.lpf_cfreq = MPU9250_LPF_CFREQ;
    sensor_cfg.sample_rate = MPU9250_SAMPLE_RATE;

    struct mpu9250_dmp_cfg dmp_cfg = {0};

    dmp_cfg.feat_mask = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL |
            DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_SEND_RAW_ACCEL;
    dmp_cfg.sample_rate = MPU9250_DMP_SAMPLE_RATE;

    int err = mobile_mpu9250_init(&sensor_cfg, &dmp_cfg);

    if (err != 0) {
        printk("Initialisation unsuccessful: error %d\n", err);
        return;
    }

    while (1) {
        if ( fifo_available() && dmp_update_fifo() == INV_SUCCESS) {
            struct mpu9250_sample sample;
            
            mpu9250_get_sample(&sample);
            mpu9250_print_sample(&sample);

            if (ble_connected()) {
                struct packet msg;
                mpu9250_format_notify_msg(&msg, &sample);
                k_msgq_put(&notify_msgq, &msg, K_NO_WAIT);
			}
        }
        k_msleep(10);
    }

}