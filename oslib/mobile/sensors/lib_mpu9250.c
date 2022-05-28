#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <toolchain.h>
#include <logging/log.h>
#include <drivers/i2c.h>
#include <math.h>

#include <sys/printk.h>

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu9250_registers.h"
#include "lib_mpu9250.h"

struct mpu9250_config {
    const struct i2c_dt_spec i2c;
    uint8_t gyro_sr_div;
    uint8_t gyro_dlpf;
    uint8_t gyro_fs;
    uint8_t accel_fs;
    uint8_t accel_dlpf;
};

struct pwr_ctrl_cfg {
    const char *port;
    uint32_t pin;
};

enum t_axisOrder {
    X_AXIS, // 0
    Y_AXIS, // 1
    Z_AXIS  // 2
};

#ifndef MIN
#define MIN(x, y) ((x) > (y) ? (y) : (x))
#endif

#ifndef MAX
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#endif

#ifndef CONSTRAIN
#define CONSTRAIN(x, a, b) (MIN(MAX(x, a), b))
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif

#define MPU9250_VDD_PWR_CTRL_INIT_PRIORITY 85
#define MAX_DMP_SAMPLE_RATE 200

LOG_MODULE_REGISTER(lib_mpu9250_log, LOG_LEVEL_DBG);

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
long qw, qx, qy, qz;
long temperature;
unsigned long time;
float pitch, roll, yaw;
float heading;

static struct device* mpu9250_i2c_dev = NULL;
static float gyro_sens = 0;
static float accel_sens = 0;
static float magn_sens = 6.665f;

static inline float get_gyro_sens();
static inline float get_accel_sens();
static inline uint8_t dmp_init(uint16_t features, uint8_t fifo_rate);

static inline int mpu9250_set_gyro_fsr(uint16_t gyro_fsr);
static inline int mpu9250_set_accel_fsr(uint8_t accel_fsr);

int mpu9250_i2c_write(unsigned char dev_addr, unsigned char reg_addr,
                      unsigned char length, unsigned char * data) {
    return i2c_burst_write(mpu9250_i2c_dev, dev_addr, reg_addr, data, length);
}

int mpu9250_i2c_read(unsigned char dev_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data) {
    return i2c_burst_read(mpu9250_i2c_dev, dev_addr, reg_addr, data, length);
}

void mpu9250_get_ms(unsigned long *count) {

    *count = k_uptime_get();

}

void mpu9250_delay_ms(unsigned long ms) {

    k_msleep(ms);

}

static inline int mpu9250_set_gyro_fsr(uint16_t gyro_fsr) {
    int8_t err;
    err = mpu_set_gyro_fsr(gyro_fsr);
    if (err == INV_SUCCESS)
    {
        gyro_sens = get_gyro_sens();
    }
    return err;
}


static inline int mpu9250_set_accel_fsr(uint8_t accel_fsr) {
    int8_t err;
    err = mpu_set_accel_fsr(accel_fsr);
    if (err == INV_SUCCESS)
    {
        accel_sens = get_accel_sens();
    }
    return err;
}

inline float get_gyro_sens() {
    float sens;
    if (mpu_get_gyro_sens(&sens) == INV_SUCCESS) {
        return sens;
    }
    return 0.0/0.0;
}

inline float get_accel_sens() {
    uint16_t sens;
    if (mpu_get_accel_sens(&sens) == INV_SUCCESS) {
        return sens;
    }
    return 0.0/0.0;
}

uint8_t dmp_enable_features(uint16_t mask) {
    uint16_t enMask = 0;
    enMask |= mask;
    // Combat known issue where fifo sample rate is incorrect
    // unless tap is enabled in the DMP.
    enMask |= DMP_FEATURE_TAP; 
    return dmp_enable_feature(enMask);
}

int mobile_mpu9250_init(struct mpu9250_sens_cfg* sensor_cfg,
        struct mpu9250_dmp_cfg* dmp_cfg) {
    mpu9250_i2c_dev = device_get_binding(DT_LABEL(DT_NODELABEL(i2c0)));

    if (mpu9250_i2c_dev == NULL) {
        printk("No device found\n");
        return -1;
    }
    if (!device_is_ready(mpu9250_i2c_dev)) {
        printk("Device %s is not ready\n", mpu9250_i2c_dev->name);
        return -2;
    }

    printk("MPU9250 device binding retrieval successful!\n");
    
    if (mpu_init(NULL) != 0) {
        printk("MPU9250 initialisation failed!\n");
        return -3;
    }

    printk("MPU9250 initialisation successful!\n");
    
    // Place all slaves (including compass) on primary bus
    if(mpu_set_bypass(1)) {
        printk("Setting MPU9250 bypass mode failed!");
        return -4;
    }

    if(mpu_set_sensors(sensor_cfg->feat_mask)) {
        printk("Enabling Sensors failed!");
        return -5;
    }

    /*if(mpu_configure_fifo(sensor_cfg->feat_mask)) {
        printk("Enabling Sensors failed!");
        return -6;
    }*/
    
    gyro_sens = get_gyro_sens();
    accel_sens = get_accel_sens();

    if(mpu9250_set_gyro_fsr(sensor_cfg->gyro_fsr)) {
        printk("Setting Gyro FSR failed!");
        return -7;
    }

    if(mpu9250_set_accel_fsr(sensor_cfg->accel_fsr)) {
        printk("Setting Accelerometer FSR failed!");
        return -8;
    }
    mpu_set_lpf(sensor_cfg->lpf_cfreq);
    //mpu_set_sample_rate(CONSTRAIN(sensor_cfg->sample_rate, 4, 1000));
    //mpu_set_compass_sample_rate(CONSTRAIN(sensor_cfg->sample_rate, 1, 100));

    if(dmp_init(dmp_cfg->feat_mask, dmp_cfg->sample_rate)) {
        printk("MPU9250 dmp initialisation unsuccessful!");
        return -9;
    }

    printk("MPU9250 dmp initialisation successful!");
    
    return 0;

}

inline uint8_t dmp_init(uint16_t features, uint8_t fifo_rate) {
    unsigned short feat = features;
    unsigned short rate = fifo_rate;

    if (dmp_load_motion_driver_firmware() != INV_SUCCESS) {
        return INV_ERROR;
    }
    
    if (dmp_enable_features(feat) != INV_SUCCESS) {
        return INV_ERROR;
    }
    
    rate = CONSTRAIN(rate, 1, MAX_DMP_SAMPLE_RATE);
    
    if (dmp_set_fifo_rate(rate) != INV_SUCCESS) {
        return INV_ERROR;
    }
    
    return mpu_set_dmp_state(1);
}


uint8_t fifo_available() {
    uint8_t fifo_h, fifo_l;
    
    if (mpu_read_reg(MPU9250_FIFO_COUNTH, &fifo_h) != INV_SUCCESS) {
        return 0;
    }
    if (mpu_read_reg(MPU9250_FIFO_COUNTL, &fifo_l) != INV_SUCCESS) {
        return 0;
    }
    
    return (fifo_h << 8 ) | fifo_l;
}

int8_t dmp_update_fifo() {
    short gyro[3];
    short accel[3];
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    
    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)
           != INV_SUCCESS) {
       return INV_ERROR;
    }
    
    if (sensors & INV_XYZ_ACCEL) {
        ax = accel[X_AXIS];
        ay = accel[Y_AXIS];
        az = accel[Z_AXIS];
    }

    if (sensors & INV_XYZ_GYRO) {
        gx = gyro[X_AXIS];
    }
    if (sensors & INV_Y_GYRO){
        gy = gyro[Y_AXIS];
    }
    if (sensors & INV_Z_GYRO){
        gz = gyro[Z_AXIS];
    }

    if (sensors & INV_WXYZ_QUAT) {
        qw = quat[0];
        qx = quat[1];
        qy = quat[2];
        qz = quat[3];
    }
    
    time = timestamp;
    
    return INV_SUCCESS;
}

static inline float q_to_float(long number, unsigned char q)
{
    unsigned long mask = 0;
    for (int i=0; i<q; i++) {
        mask |= (1<<i);
    }
    return (number >> q) + ((number & mask) / (float) (2<<(q-1)));
}

static inline float calc_quat(long axis) {
    return q_to_float(axis, 30);
}

static inline float calc_accel(int axis) {
    return (float) axis / (float) accel_sens;
}

static inline float calc_gyro(int axis) {
    return (float) axis / (float) gyro_sens;
}

static inline float calc_magn(int axis) {
    return (float) axis / (float) magn_sens;
}

void print_imu_data() {
    // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
    // are all updated.
    // Quaternion values are, by default, stored in Q30 long
    // format. calcQuat turns them into a float between -1 and 1
    float q0 = calc_quat(qw);
    float q1 = calc_quat(qx);
    float q2 = calc_quat(qy);
    float q3 = calc_quat(qz);

    float a_x = calc_accel(ax);
    float a_y = calc_accel(ay);
    float a_z = calc_accel(az);
    float g_x = calc_gyro(gx);
    float g_y = calc_gyro(gy);
    float g_z = calc_gyro(gz);

    printk("G_r xyz: %d, %d, %d\n", gx, gy, gz);
    printk("A_r xyz: %d, %d, %d\n", ax, ay, az);
    printk("Q_r: %ld, %ld, %ld, %ld\n", qw, qx, qy, qz);
    printk("Q: %.4f, %.4f, %.4f, %.4f\n", q0, q1, q2, q3);
    printk("A xyz: %.4f, %.4f, %.4f\n", a_x, a_y, a_z);
    printk("G xyz: %.4f, %.4f, %.4f\n\n", g_x, g_y, g_z);

}

/**
 * @brief Generic power control function attained from
 *          thingy52/board.c
 * 
 * @param dev device handles
 * @return int return ERR val
 */
static int mpu_ctrl_init(const struct device *dev) {
    const struct pwr_ctrl_cfg *cfg = dev->config;
    const struct device *gpio;

    gpio = device_get_binding(cfg->port);
    if (!gpio) {
        LOG_ERR("Could not bind device \"%s\"\n", cfg->port);
        return -ENODEV;
    }

    gpio_pin_configure(gpio, cfg->pin, GPIO_OUTPUT_HIGH);
    LOG_DBG("IMU control GPIO configured");
    // Delay is required for the rail to stabalize
    k_msleep(100);
    LOG_DBG("IMU power supply nominally stabilsed");
    return 0;
}

// Power up Thingy52 MPU9250 (analog switch) using SX1509B
#define MPU6050_VDD_PWR_CTRL_GPIO_PIN 8
static const struct pwr_ctrl_cfg mpu9250_vdd_pwr_ctrl_cfg = {
    .port = DT_LABEL(DT_INST(0, semtech_sx1509b)),
    .pin = MPU6050_VDD_PWR_CTRL_GPIO_PIN,
};

DEVICE_DEFINE(mpu9250_vdd_pwr_ctrl_initx, "", mpu_ctrl_init, NULL, NULL,
              &mpu9250_vdd_pwr_ctrl_cfg, POST_KERNEL,
              MPU9250_VDD_PWR_CTRL_INIT_PRIORITY, NULL);