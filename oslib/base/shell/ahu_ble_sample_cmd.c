 /** 
 **************************************************************
 * @file ahu_ble_sample_cmd.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief Bluetooth 'sample' command Header file
 ***************************************************************
 */

#include "lib_shell.h"
#include <stdint.h>
#include <logging/log.h>
#include "tsk_ble.h"
#include <stdlib.h>
#include <errno.h>

LOG_MODULE_REGISTER(sample_cmd_log, LOG_LEVEL_DBG);

static void cmd_sample_write(const struct shell* shell, size_t argc,
        char** argv);

SHELL_STATIC_SUBCMD_SET_CREATE(sample_level_1,
        SHELL_CMD(w, NULL, "Write an interval between 10 and 300"
                " seconds inclusive", cmd_sample_write),
        SHELL_SUBCMD_SET_END
); 

/* Creating root (level 0) command "time" */
SHELL_CMD_REGISTER(sample, &sample_level_1,
        "Sets the SCU's continuous sampling interval", NULL);


static void cmd_sample_write(const struct shell* shell, size_t argc,
        char** argv) {
    
    if(argc != 2) {
        LOG_ERR("Incorrect argument count: expected 2, found %d", argc);
        return;
    }

    const char *nptr = argv[1];
    char* endptr;

    /* reset errno to 0 before call */

    errno = 0;

    uint16_t interval = (uint16_t)strtoul(argv[1], &endptr, 10);

    if(errno == 0 && interval >= 10 && interval <= 300) {
        ble_ctrl_request_packet_t msg = {ble_sample, 0,
        "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};

        strncpy(msg.payload, argv[1], (uint8_t)(endptr - nptr));
        msg.payload_length = (uint8_t)(endptr - nptr);
        
        ble_ctrl_request_send(&msg);
    } else {
        LOG_ERR("Invalid sample interval");
    }

}