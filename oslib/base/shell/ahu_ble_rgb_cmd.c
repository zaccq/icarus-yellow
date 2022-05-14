 /** 
 **************************************************************
 * @file ahu_ble_rgb_cmd.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief Bluetooth rgb command Header file
 ***************************************************************
 */

#include "lib_shell.h"
#include <stdint.h>
#include <logging/log.h>
#include "tsk_ble.h"

#define OPTIONS_STRING "\n\xff\rOptions:\n\xff\r  o  :Turn LED on\n  f  :Turn LED off\n  t  :Toggle LED"

LOG_MODULE_REGISTER(rgb_cmd_log, LOG_LEVEL_DBG);

static int cmd_ble_rgb_led_write(const struct shell* shell, size_t argc,
        char** argv, void *data);

SHELL_SUBCMD_DICT_SET_CREATE(rgb_level_3, cmd_ble_rgb_led_write,
        (f, 0), (o, 1), (t, 2)
);

SHELL_STATIC_SUBCMD_SET_CREATE(rgb_level_2,
        SHELL_CMD(r,   &rgb_level_3, "Red RGB sub-LED.", NULL),
        SHELL_CMD(g,   &rgb_level_3, "Green RGB sub-LED", NULL),
        SHELL_CMD(b,   &rgb_level_3, "Blue RGB sub-LED" 
                OPTIONS_STRING /*Because apparently you're not allowed to
                document your dictionary commands*/,
                NULL),
        SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(rgb_level_1,
        SHELL_CMD(w, &rgb_level_2, "Write command",
            NULL),
        SHELL_SUBCMD_SET_END
); 

/* Creating root (level 0) command "time" */
SHELL_CMD_REGISTER(rgb, &rgb_level_1, "Sets the SCU's onboard LEDs", NULL);


static int cmd_ble_rgb_led_write(const struct shell* shell, size_t argc,
        char** argv, void* data) {
    
    ble_ctrl_request_packet_t msg = {ble_rgb_led, 2,
        "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};

    msg.payload[0] = argv[-1][0]; //parent commands may be accessed with negative indices
    msg.payload[1] = ((int)data&0xff) + '0';
    
    ble_ctrl_request_send(&msg);

    return 0;

}