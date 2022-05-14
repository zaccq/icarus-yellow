 /** 
 **************************************************************
 * @file ahu_ble_buzzer_cmd.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief Bluetooth buzzer command Header file
 ***************************************************************
 */

#include "lib_shell.h"
#include <stdint.h>
#include <logging/log.h>
#include "tsk_ble.h"
#include <stdlib.h>
#include <errno.h>

LOG_MODULE_REGISTER(buzzer_log_module, LOG_LEVEL_DBG);

static void cmd_buzzer_write_freq(const struct shell* shell, size_t argc,
        char** argv);

static void cmd_buzzer_write_off(const struct shell* shell, size_t argc,
char** argv);

SHELL_STATIC_SUBCMD_SET_CREATE(buzzer_level_2_cmd,
        SHELL_CMD(freq, NULL, "Turn on buzzer and write frequency between 100"
                " and 22000 Hz",
                cmd_buzzer_write_freq),
        SHELL_CMD(off, NULL, "Turn Buzzer off", cmd_buzzer_write_off),
        SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(buzzer_level_1_cmd,
        SHELL_CMD(w, &buzzer_level_2_cmd, "Write command", NULL),
        SHELL_SUBCMD_SET_END
);

/* Creating root (level 0) command "time" */
SHELL_CMD_REGISTER(buzzer, &buzzer_level_1_cmd, "Interfaces with the SCU's onboard " 
    "buzzer.", NULL);


static void cmd_buzzer_write_freq(const struct shell* shell, size_t argc,
        char** argv) {

    for(uint8_t i = 0; i < argc; i++) {
        LOG_DBG("argv[%d]: %s", i, argv[i]);
    }

    if(argc != 2) {
        LOG_ERR("Incorrect argument count: expected 2, found %d", argc);
        return;
    }

    const char *nptr = argv[1];
    char* endptr;

    /* reset errno to 0 before call */
    errno = 0;

    uint16_t freq = (uint16_t)strtoul(argv[1], &endptr, 10);

    if(errno == 0 && freq >= 20 && freq <= 22000) {
        ble_ctrl_request_packet_t msg = {ble_buzzer, 0,
        "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};

        strncpy(msg.payload, argv[1], (uint8_t)(endptr - nptr));
        msg.payload_length = (uint8_t)(endptr - nptr);
        
        ble_ctrl_request_send(&msg);
    }  else {
        LOG_ERR("Invalid frequency");
    }

}

static void cmd_buzzer_write_off(const struct shell* shell, size_t argc,
        char** argv) {
    
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    ble_ctrl_request_packet_t msg = {ble_buzzer, 0,
    "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};

    msg.payload[0] = '0';
    msg.payload_length = 1;
    
    ble_ctrl_request_send(&msg);

}