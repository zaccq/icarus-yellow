#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <toolchain.h>
#include <timing/timing.h>
#include "ultra.h"

#include <bluetooth/bluetooth.h>

/* DeviceTree get node ID from label */
#define GPIO0 DT_NODELABEL(gpio0)
#define GPIO0_13 0x0D //PIN PO.13

/* Define the trig and echo GPIO pins */
#define GPIO_01 0x01 // PIN P1.01 - Echo(Input) (D2)
#define GPIO_02 0x02 // PIN P1.02 - Trig(Output) (D3)

#define GPIO_ECHO GPIO_01
#define GPIO_TRIG GPIO_02

#define STACK_SIZE 1024
#define PRIORITY 3

#define ULTRA_MSG_SEM_MAX_COUNT 1
#define ULTRA_MSG_SEM_INIT_COUNT 1
#define SEMAPHORE_TAKEN 0

K_SEM_DEFINE(ultra_msg_sem, ULTRA_MSG_SEM_INIT_COUNT, ULTRA_MSG_SEM_MAX_COUNT);
K_MSGQ_DEFINE(ultra_q, sizeof(struct ultra_reading), 5, 4);

static struct gpio_callback echo_cb_data;
const static struct device * dev_gpio1;


volatile uint32_t pulse_start = 0;
volatile uint32_t pulse_end = 0;

void process_pulse(const struct device *dev_gpio);
void set_ultra(const struct device *dev_gpio);

void on_echo(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins) {
    
    if (gpio_pin_get(dev_gpio1, GPIO_ECHO) == 1) {

        pulse_start = k_cycle_get_32();

    } else {

        pulse_end = k_cycle_get_32();
        
        uint32_t dist = k_cyc_to_us_ceil64(pulse_end - pulse_start)/58;
        uint8_t msg = dist & 0xFF;
        k_msgq_put(&ultra_q, &msg, K_NO_WAIT);

    }
    
}

void init_untra(const struct device *dev_gpio) {

    // In here we want to initialise the pins as input and output
    int ret = gpio_pin_configure(dev_gpio, GPIO_ECHO, GPIO_INPUT);

    // Checking the configuration was all good
    if (ret < 0) {
        return;
    }

    ret = gpio_pin_configure(dev_gpio, GPIO_TRIG, GPIO_OUTPUT_ACTIVE);

    // Tested and all good
    if (ret < 0) {
        return;
    }

    ret = gpio_pin_interrupt_configure(dev_gpio, GPIO_ECHO,
            GPIO_INT_EDGE_BOTH);
    
    if (ret != 0) {
		printk("Error %d: failed to configure interrupt\n", ret);
		return;
	}

    gpio_init_callback(&echo_cb_data, on_echo, BIT(GPIO_ECHO));
	gpio_add_callback(dev_gpio, &echo_cb_data);

    
}

void process_pulse(const struct device *dev_gpio) {

	uint64_t total_cycles;
	uint32_t start_time;
	uint32_t end_time;

	while (gpio_pin_get(dev_gpio, GPIO_ECHO) == 0) {}

	start_time = k_cycle_get_32();

	while (gpio_pin_get(dev_gpio, GPIO_ECHO) == 1) {}

    end_time = k_cycle_get_32() - start_time;

	total_cycles = k_cyc_to_us_ceil32(end_time);
	printk("Range is %" PRIu64 "\n", (total_cycles/58));

}

void set_ultra(const struct device *dev_gpio) {

	// Set the trigger to high to generate pulse
	gpio_pin_set(dev_gpio, GPIO_TRIG, 1);
	k_usleep(10);
	gpio_pin_set(dev_gpio, GPIO_TRIG, 0);

	// Using Echo pin, find the change in time and then range in cm
	//process_pulse(dev_gpio);
}

void ultra_threading(void *a, void *b, void *c) {

    dev_gpio1 = device_get_binding("GPIO_1"); // For the Ultrasonic sensor

    if (dev_gpio1 == NULL) {
        return;
    }

    init_untra(dev_gpio1);

    while(1) {
        set_ultra(dev_gpio1);
        k_msleep(200);
    }

}

K_THREAD_DEFINE(ultra_id, STACK_SIZE, ultra_threading,
				 NULL, NULL, NULL, PRIORITY, 0, 0);
