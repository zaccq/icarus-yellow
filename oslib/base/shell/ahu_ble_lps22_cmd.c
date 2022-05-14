 /** 
 **************************************************************
 * @file ahu_ble_lps22_cmd.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief Bluetooth lps22 command Header file
 ***************************************************************
 */

#include "lib_shell.h"
#include <stdint.h>
#include <logging/log.h>
#include "tsk_ble.h"

LOG_MODULE_REGISTER(lps22_cmd_log, LOG_LEVEL_DBG);

static void cmd_lps22_pressure_read(const struct shell* shell, size_t argc,
        char** argv);

SHELL_STATIC_SUBCMD_SET_CREATE(lps22_level_1,
        SHELL_CMD(r, NULL, "Read ambient pressure value",
            cmd_lps22_pressure_read),
        SHELL_SUBCMD_SET_END
); 

/* Creating root (level 0) command "time" */
SHELL_CMD_REGISTER(lps22, &lps22_level_1, "Interfaces with the SCU's onboard " 
    "lps22 barometer.", NULL);


static void cmd_lps22_pressure_read(const struct shell* shell, size_t argc,
        char** argv) {
    
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    ble_ctrl_request_packet_t msg = {ble_lps22_pressure, 0,
        "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};
    
    ble_ctrl_request_send(&msg);

}