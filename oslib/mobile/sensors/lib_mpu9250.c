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


int ax, ay, az;
int gx, gy, gz;
int mx, my, mz;
long qw, qx, qy, qz;
long temperature;
unsigned long time;
float pitch, roll, yaw;
float heading;

static struct device* mpu9250_i2c_dev = NULL;

static float gyro_sens;
static float accel_sens;

static inline float get_gyro_sens();
static inline float get_accel_sens();
int mobile_mpu9250_init(uint8_t sensors, uint8_t dmp_features, uint8_t dmp_sample_rate);
static inline uint8_t dmp_init(uint8_t features, uint8_t fifo_rate);

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

inline float get_gyro_sens() {
    float sens;
	if (mpu_get_gyro_sens(&sens) == INV_SUCCESS) {
		return sens;
	}
    return 0.0/0.0;
}

inline float get_accel_sens() {
    float sens;
	if (mpu_get_gyro_sens(&sens) == INV_SUCCESS) {
		return sens;
	}
    return 0.0/0.0;
}

uint8_t dmp_enable_features(uint8_t mask) {
	uint8_t enMask = 0;
	enMask |= mask;
	// Combat known issue where fifo sample rate is incorrect
	// unless tap is enabled in the DMP.
	enMask |= DMP_FEATURE_TAP; 
	return dmp_enable_feature(enMask);
}

int mobile_mpu9250_init(uint8_t sensors, uint8_t dmp_features, uint8_t dmp_sample_rate) {
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

    if(mpu_set_sensors(sensors)) {
        printk("Enabling MPU9250 sensors failed!");
        return -5;
    }
	
	gyro_sens = get_gyro_sens();
	accel_sens = get_accel_sens();

    if(dmp_init(dmp_features, dmp_sample_rate)) {
        printk("MPU9250 dmp initialisation unsuccessful!");
        return -6;
    }

    printk("MPU9250 dmp initialisation successful!");
	
	return 0;

}

inline uint8_t dmp_init(uint8_t features, uint8_t fifo_rate) {
    unsigned short feat = features;
	unsigned short rate = fifo_rate;

	if (dmp_load_motion_driver_firmware() != INV_SUCCESS)
		return INV_ERROR;
	
	// 3-axis and 6-axis LP quat are mutually exclusive.
	// If both are selected, default to 3-axis
	if (feat & DMP_FEATURE_6X_LP_QUAT) {

		feat &= ~(DMP_FEATURE_LP_QUAT);
		dmp_enable_6x_lp_quat(1);

	} else if (feat & DMP_FEATURE_LP_QUAT) {
		dmp_enable_lp_quat(1);
    }

	if (feat & DMP_FEATURE_GYRO_CAL) {
        dmp_enable_gyro_cal(1);
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
	
	if (mpu_read_reg(MPU9250_FIFO_COUNTH, &fifo_h) != INV_SUCCESS)
		return 0;
	if (mpu_read_reg(MPU9250_FIFO_COUNTL, &fifo_l) != INV_SUCCESS)
		return 0;
	
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
		   != INV_SUCCESS)
    {
	   return INV_ERROR;
    }
	
	if (sensors & INV_XYZ_ACCEL)
	{
		ax = accel[X_AXIS];
		ay = accel[Y_AXIS];
		az = accel[Z_AXIS];
	}
	if (sensors & INV_X_GYRO)
		gx = gyro[X_AXIS];
	if (sensors & INV_Y_GYRO)
		gy = gyro[Y_AXIS];
	if (sensors & INV_Z_GYRO)
		gz = gyro[Z_AXIS];
	if (sensors & INV_WXYZ_QUAT)
	{
		qw = quat[0];
		qx = quat[1];
		qy = quat[2];
		qz = quat[3];
	}
	
	time = timestamp;
	
	return INV_SUCCESS;
}

float q_to_float(long number, unsigned char q)
{
	unsigned long mask = 0;
	for (int i=0; i<q; i++)
	{
		mask |= (1<<i);
	}
	return (number >> q) + ((number & mask) / (float) (2<<(q-1)));
}

float calc_quat(long axis) {
	return q_to_float(axis, 30);
}

void compute_euler_angles(uint8_t degrees) {
    float dqw = calc_quat(qw);
    float dqx = calc_quat(qx);
    float dqy = calc_quat(qy);
    float dqz = calc_quat(qz);
    
    float ysqr = dqy * dqy;
    float t0 = -2.0f * (ysqr + dqz * dqz) + 1.0f;
    float t1 = +2.0f * (dqx * dqy - dqw * dqz);
    float t2 = -2.0f * (dqx * dqz + dqw * dqy);
    float t3 = +2.0f * (dqy * dqz - dqw * dqx);
    float t4 = -2.0f * (dqx * dqx + ysqr) + 1.0f;
  
	// Keep t2 within range of asin (-1, 1)
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
  
    pitch = asin(t2) * 2;
    roll = atan2(t3, t4);
    yaw = atan2(t1, t0);
	
	if (degrees) {
		pitch *= (180.0 / PI);
		roll *= (180.0 / PI);
		yaw *= (180.0 / PI);
		if (pitch < 0) pitch = 360.0 + pitch;
		if (roll < 0) roll = 360.0 + roll;
		if (yaw < 0) yaw = 360.0 + yaw;	
	}
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

    printk("Q: %.4f, %.4f, %.4f, %.4f\n", q0, q1, q2, q3);
    printk("R/P/Y: %.2f, %.2f, %.2f\n\n", roll, pitch, yaw);

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