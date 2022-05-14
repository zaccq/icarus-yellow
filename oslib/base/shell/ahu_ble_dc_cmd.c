 /** 
 **************************************************************
 * @file ahu_ble_dc_cmd.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief Bluetooth duty cycle command Header file
 ***************************************************************
 */

#include "lib_shell.h"
#include <stdint.h>
#include <logging/log.h>
#include "tsk_ble.h"
#include <stdlib.h>
#include <errno.h>

LOG_MODULE_REGISTER(dc_cmd_log, LOG_LEVEL_DBG);

static void cmd_dc_write(const struct shell* shell, size_t argc,
        char** argv);

SHELL_STATIC_SUBCMD_SET_CREATE(dc_level_1,
        SHELL_CMD(w, NULL, "Write a duty cycle between 1 and 100 inclusive",
            cmd_dc_write),
        SHELL_SUBCMD_SET_END
); 

/* Creating root (level 0) command "time" */
SHELL_CMD_REGISTER(dc, &dc_level_1, "Sets the SCU's awake duty cycle", NULL);


static void cmd_dc_write(const struct shell* shell, size_t argc, char** argv) {
    
    if(argc != 2) {
        LOG_ERR("Incorrect argument count: expected 2, found %d", argc);
        return;
    }

    const char *nptr = argv[1];
    char* endptr;

    /* reset errno to 0 before call */

    errno = 0;

    uint16_t duty_cycle = (uint16_t)strtoul(argv[1], &endptr, 10);

    if(errno == 0 && duty_cycle > 0 && duty_cycle <= 100) {
        ble_ctrl_request_packet_t msg = {ble_dc, 0,
        "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};

        strncpy(msg.payload, argv[1], (uint8_t)(endptr - nptr));
        msg.payload_length = (uint8_t)(endptr - nptr);
        
        ble_ctrl_request_send(&msg);
    } else {
        LOG_ERR("Invalid duty cycle value");
    }

}