 /** 
 **************************************************************
 * @file ahu_ble_hts221_cmd.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief Bluetooth hts221 command Header file
 ***************************************************************
 */

#include "lib_shell.h"
#include <stdint.h>
#include <logging/log.h>
#include "tsk_ble.h"

LOG_MODULE_REGISTER(hts221_cmd_log, LOG_LEVEL_DBG);

static void cmd_hts221_temp_read(const struct shell* shell, size_t argc,
        char** argv);

static void cmd_hts221_humidity_read(const struct shell* shell, size_t argc,
        char** argv);

SHELL_STATIC_SUBCMD_SET_CREATE(hts221_level_2,
        SHELL_CMD(t,   NULL, "Temperature", cmd_hts221_temp_read),
        SHELL_CMD(h,   NULL, "Humidity", cmd_hts221_humidity_read),
        SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(hts221_level_1,
        SHELL_CMD(r,   &hts221_level_2, "Read parameter", NULL),
        SHELL_SUBCMD_SET_END
); 

/* Creating root (level 0) command "time" */
SHELL_CMD_REGISTER(hts221, &hts221_level_1, "Interfaces with the SCU's onboard " 
    "HTS221 temperature and humidity sensor.", NULL);


static void cmd_hts221_temp_read(const struct shell* shell, size_t argc,
        char** argv) {
    
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    ble_ctrl_request_packet_t msg = {ble_hts221_temp, 0,
        "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};
    
    ble_ctrl_request_send(&msg);

}

static void cmd_hts221_humidity_read(const struct shell* shell, size_t argc,
        char** argv) {
    
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    ble_ctrl_request_packet_t msg = {ble_hts221_humidity, 0,
        "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};
    
    ble_ctrl_request_send(&msg);

}