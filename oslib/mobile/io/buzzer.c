/*
***************************************************************************
* @file oslib/scu_drivers/scu_io/scu_buzzer.c
* @author Matthew Kumar - 45303494
* @date 1-04-2021
* @brief Buzzer functionalitiy
***************************************************************************
* EXTERNAL FUNCTIONS
***************************************************************************
* void buzzer_init() - Function inits the buzzer
***************************************************************************
*/

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>
#include <drivers/regulator.h>
#include <drivers/pwm.h>
#include <logging/log.h>

#include "buzzer.h"

#define STACK_SIZE 1024
#define PRIORITY 3
#define SPK_PWR_NODE DT_NODELABEL(spk_pwr)

LOG_MODULE_REGISTER(buz_log_module, LOG_LEVEL_DBG);

K_MSGQ_DEFINE(buzzer_queue, sizeof(struct buzzer_freq), 10, 4);

static const struct device* speaker_power = DEVICE_DT_GET(SPK_PWR_NODE);
const struct device *pwm_dev;


/**
 * @brief Function inits the buzzer
 */
void buzzer_init(void) {

    pwm_dev = device_get_binding(DT_LABEL(DT_NODELABEL(pwm0)));

    if (!device_is_ready(pwm_dev)) {
		printk("E");
        LOG_DBG("FREQ is dev not working");
		return;
	}

	if (!device_is_ready(speaker_power)) {
		printk("Error");
        LOG_DBG("SPEAKER NOT WORKING");
		return;
	}

    regulator_enable(speaker_power, NULL);

}

/**
 * @brief Function gets the buzzer with the given frequency
 * 
 * @param freq the frequency the buzzer is set at
 */
void buzzer_start(int freq) {

    if (freq == 0) {
        pwm_pin_set_usec(pwm_dev, 0x1b, 0,0,0);
    } else {

        pwm_pin_set_usec(pwm_dev, 0x1b, (USEC_PER_SEC/freq), ((USEC_PER_SEC/freq) / 2U), 0);   
    }
}
/**
 * @brief Thread entry point for the buzzer
 * 
 * @param a void pointer
 * @param b void pointer
 * @param c void pointer
 */
void thread_buzzer(void *a, void *b, void *c) {

    struct buzzer_freq buzzerData;

    while (1) {
       if (k_msgq_get(&buzzer_queue, &buzzerData, K_FOREVER) == 0) {
           buzzer_start(buzzerData.freq);
       }
    }

}

K_THREAD_DEFINE(buzzer_id, STACK_SIZE, thread_buzzer,
		NULL, NULL, NULL, PRIORITY, 0, 0);

