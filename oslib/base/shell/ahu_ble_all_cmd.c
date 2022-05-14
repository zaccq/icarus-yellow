 /** 
 **************************************************************
 * @file ahu_ble_all_cmd.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief Bluetooth all command Header file
 ***************************************************************
 */

#include "lib_shell.h"
#include <stdint.h>
#include <logging/log.h>
#include "tsk_ble.h"

LOG_MODULE_REGISTER(all_cmd_log, LOG_LEVEL_DBG);

static void cmd_all_read(const struct shell* shell, size_t argc,
        char** argv);

SHELL_STATIC_SUBCMD_SET_CREATE(all_level_1,
        SHELL_CMD(o, NULL, "Enable sampling of all sensors", cmd_all_read),
        SHELL_CMD(f, NULL, "Disable sampling of all sensors", cmd_all_read),
        SHELL_SUBCMD_SET_END
); 

/* Creating root (level 0) command "time" */
SHELL_CMD_REGISTER(all, &all_level_1, "Reads from all SCU sensors and "
        "outputs the results in JSON", NULL);


static void cmd_all_read(const struct shell* shell, size_t argc,
        char** argv) {
    
    if(argc != 1) {
        LOG_ERR("Incorrect argument count: expected 1, found %d", argc);
        return;
    }

    ble_ctrl_request_packet_t msg = {ble_all, 0,
        { 0 }};

    if(argv[0][0] == 'o') {
        msg.payload_length = 1;
        msg.payload[0] = '1';
    } else if(argv[0][0] == 'f') {
        msg.payload_length = 1;
        msg.payload[0] = '0';
    } else {
        return;
    }

    ble_ctrl_request_send(&msg);

}

