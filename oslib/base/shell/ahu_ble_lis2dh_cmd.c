 /** 
 **************************************************************
 * @file ahu_ble_lis2dh_cmd.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief Bluetooth lis2dh command Header file
 ***************************************************************
 */

#include "lib_shell.h"
#include <stdint.h>
#include <logging/log.h>
#include "tsk_ble.h"

LOG_MODULE_REGISTER(lis2dh_cmd_log, LOG_LEVEL_DBG);

static void cmd_lis2dh_x_read(const struct shell* shell, size_t argc,
        char** argv);

static void cmd_lis2dh_y_read(const struct shell* shell, size_t argc,
        char** argv);

static void cmd_lis2dh_z_read(const struct shell* shell, size_t argc,
        char** argv);

SHELL_STATIC_SUBCMD_SET_CREATE(lis2dh_level_2,
        SHELL_CMD(x,   NULL, "X-axis acceleration value", cmd_lis2dh_x_read),
        SHELL_CMD(y,   NULL, "Y-axis acceleration value", cmd_lis2dh_y_read),
        SHELL_CMD(z,   NULL, "Y-axis acceleration value", cmd_lis2dh_z_read),
        SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(lis2dh_level_1,
        SHELL_CMD(r,   &lis2dh_level_2, "Read parameter", NULL),
        SHELL_SUBCMD_SET_END
); 

/* Creating root (level 0) command "time" */
SHELL_CMD_REGISTER(lis2dh, &lis2dh_level_1, "Interfaces with the SCU's onboard " 
    "LIS2DH 3-axis accelerometer.", NULL);


static void cmd_lis2dh_x_read(const struct shell* shell, size_t argc,
        char** argv) {
    
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    ble_ctrl_request_packet_t msg = {ble_lis2dh_x, 0,
        "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};
    
    ble_ctrl_request_send(&msg);

}

static void cmd_lis2dh_y_read(const struct shell* shell, size_t argc,
        char** argv) {
    
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    ble_ctrl_request_packet_t msg = {ble_lis2dh_y, 0,
        "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};
    
    ble_ctrl_request_send(&msg);

}

static void cmd_lis2dh_z_read(const struct shell* shell, size_t argc,
        char** argv) {
    
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    ble_ctrl_request_packet_t msg = {ble_lis2dh_z, 0,
        "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};
    
    ble_ctrl_request_send(&msg);

}