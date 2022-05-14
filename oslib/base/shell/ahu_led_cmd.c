 /** 
 **************************************************************
 * @file ahu_led_cmd.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief AHU led command Header file
 ***************************************************************
 */

#include "lib_shell.h"
#include "tsk_led.h"
#include <stdint.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(led_cmd_log, LOG_LEVEL_DBG);

static int cmd_led_ctrl(const struct shell *shell, size_t argc, char **argv);

int8_t led_ctrl_msg_send(led_ctrl_packet_t* msg);

int8_t led_ctrl_cmd_parse(led_ctrl_packet_t* msg, int argc, char** argv);

int8_t led_ctrl_cmd_parse_led(led_ctrl_packet_t* msg, char* argv);

int8_t led_ctrl_cmd_parse_action(led_ctrl_packet_t* msg, char* argv);




SHELL_STATIC_SUBCMD_SET_CREATE(led_level_2,
        SHELL_CMD(r,   NULL, "Red RGB sub-LED", NULL),
        SHELL_CMD(g,   NULL, "Green RGB sub-LED", NULL),
        SHELL_CMD(b,   NULL, "Blue RGB sub-LED", NULL),
        SHELL_CMD(d,   NULL, "Green Led (debug only)", NULL),
        SHELL_SUBCMD_SET_END
); 

SHELL_STATIC_SUBCMD_SET_CREATE(led_level_1,
        SHELL_CMD(o,   &led_level_2, "Turns LED on", NULL),
        SHELL_CMD(f,   &led_level_2, "Turns LED off", NULL),
        SHELL_CMD(t,   &led_level_2, "Turns LEDs on", NULL),
        SHELL_SUBCMD_SET_END
); 

SHELL_CMD_REGISTER(led, &led_level_1, "Led Control (On/Off)", cmd_led_ctrl);

/******************** Shell Command Callback Functions ***********************/

static int cmd_led_ctrl(const struct shell *shell, size_t argc, char **argv){

    led_ctrl_packet_t msg = {led1_r, set_led, 1};

    if(led_ctrl_cmd_parse(&msg, argc, argv) == 0) {
        led_ctrl_msg_send(&msg);
        LOG_DBG("Led control message sent: <%d, %d, %d>", msg.led, msg.action, msg.status);
    }
    
    return 0;

}

/**************************** Helper Functions *******************************/

int8_t led_ctrl_msg_send(led_ctrl_packet_t* msg) {
    if (k_msgq_put(&led_ctrl_msgq, msg, K_NO_WAIT) != 0) {
        //Queue full, return failure
        return -1;
    }
    return 0;
}

int8_t led_ctrl_cmd_parse(led_ctrl_packet_t* msg, int argc, char** argv) {

    if(argc != 3) {
        LOG_ERR("Incorrect argument count: expected 3, found %d", argc);
        return -1;
    }

    if(led_ctrl_cmd_parse_led(msg, argv[2]) != 0 ||
    led_ctrl_cmd_parse_action(msg, argv[1]) != 0) {
        for(uint8_t i = 0; i < argc; i++) {
            LOG_DBG("argv[%d]: %s", i, argv[i]);
        }
        LOG_ERR("Invalid argument(s): try \"led -h\"");
        return -1;
    }

    return 0;

}

int8_t led_ctrl_cmd_parse_led(led_ctrl_packet_t* msg, char* argv) {
    if (strcmp(argv, "r") == 0) {
        msg->led = led1_r;
    } else if (strcmp(argv, "g") == 0) {
        msg->led = led1_g;
    } else if (strcmp(argv, "b") == 0) {
        msg->led = led1_b;
    } else if (strcmp(argv, "d") == 0) {
        msg->led = led0_g;
    } else {
        return -1;
    }
    return 0;
}

int8_t led_ctrl_cmd_parse_action(led_ctrl_packet_t* msg, char* argv) {
    if (strcmp(argv, "o") == 0) {
        msg->action = set_led;
        msg->status = 1;
    } else if (strcmp(argv, "f") == 0) {
        msg->action = set_led;
        msg->status = 0;
    } else if (strcmp(argv, "t") == 0) {
        msg->action = toggle_led;
    } else {
        return -1;
    }
    return 0;

}