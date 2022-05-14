 /** 
 **************************************************************
 * @file ahu_ble_ccs811_cmd.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief Bluetooth ccs811 command Header file
 ***************************************************************
 */

#include "lib_shell.h"
#include <stdint.h>
#include <logging/log.h>
#include "tsk_ble.h"

LOG_MODULE_REGISTER(ccs811_cmd_log, LOG_LEVEL_DBG);

static void cmd_ccs811_volatiles_read(const struct shell* shell, size_t argc,
        char** argv);

static void cmd_ccs811_co2_read(const struct shell* shell, size_t argc,
        char** argv);

SHELL_STATIC_SUBCMD_SET_CREATE(ccs811_level_2,
        SHELL_CMD(v, NULL, "Read VOC concentration", cmd_ccs811_volatiles_read),
        SHELL_CMD(c, NULL, "Read CO2 concentration", cmd_ccs811_co2_read),
        SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(ccs811_level_1,
        SHELL_CMD(r, &ccs811_level_2, "Read parameter", cmd_ccs811_volatiles_read),
        SHELL_SUBCMD_SET_END
); 

/* Creating root (level 0) command "time" */
SHELL_CMD_REGISTER(ccs811, &ccs811_level_1, "Interfaces with the SCU's onboard " 
    "ccs811 VOC sensor.", NULL);


static void cmd_ccs811_volatiles_read(const struct shell* shell, size_t argc,
        char** argv) {
    
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    ble_ctrl_request_packet_t msg = {ble_ccs811_volatiles, 0,
        "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};
    
    ble_ctrl_request_send(&msg);

}

static void cmd_ccs811_co2_read(const struct shell* shell, size_t argc,
        char** argv) {
    
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    ble_ctrl_request_packet_t msg = {ble_ccs811_co2, 0,
        "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};
    
    ble_ctrl_request_send(&msg);

}