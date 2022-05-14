 /** 
 **************************************************************
 * @file ahu_time_cmd.h
 * @author Zachary Querengasser - s4357807
 * @date 2022-03-31
 * @brief AHU time command Header file
 ***************************************************************
 */

#include "lib_shell.h"
#include "tsk_led.h"
#include <stdint.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(time_cmd_log, LOG_LEVEL_DBG);

static void cmd_time_display_seconds(const struct shell *shell, size_t argc,
        char **argv);

static void cmd_time_display_formatted(const struct shell *shell, size_t argc,
        char **argv);

void print_uptime_formatted(void);

uint8_t seconds_from_ms(int64_t ms_val);

uint8_t minutes_from_ms(int64_t ms_val);

uint8_t hours_from_ms(int64_t ms_val);

uint8_t days_from_ms(int64_t ms_val);

SHELL_STATIC_SUBCMD_SET_CREATE(time_display,
        SHELL_CMD(f,   NULL, "Format output as DDd HH:MM:SS", cmd_time_display_formatted),
        SHELL_SUBCMD_SET_END
); 

SHELL_CMD_REGISTER(time, &time_display, "Display current system timer value", cmd_time_display_seconds);

/******************** Shell Command Callback Functions ***********************/

static void cmd_time_display_seconds(const struct shell *shell, size_t argc, char **argv) {

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    printk("%lli\n", k_uptime_get()/1000);

}
static void cmd_time_display_formatted(const struct shell *shell, size_t argc, char **argv) {

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    print_uptime_formatted();
    
}

/**************************** Helper Functions *******************************/

void print_uptime_formatted(void) {
    int64_t uptime = k_uptime_get();
    printk("%02dd %02d:%02d:%02d\n", days_from_ms(uptime),hours_from_ms(uptime),
        minutes_from_ms(uptime),seconds_from_ms(uptime));
}

uint8_t seconds_from_ms(int64_t ms_val) {

    if(ms_val < 0) {
        return 0;
    }
    return (uint8_t) (ms_val / 1000);

}

uint8_t minutes_from_ms(int64_t ms_val) {

    if(ms_val < 0) {
        return 0;
    }
    return (uint8_t) (ms_val / 60000);

}

uint8_t hours_from_ms(int64_t ms_val) {

    if(ms_val < 0) {
        return 0;
    }
    return (uint8_t) (ms_val / 3600000);

}

uint8_t days_from_ms(int64_t ms_val) {

    if(ms_val < 0) {
        return 0;
    }
    return (uint8_t) (ms_val / 86400000);

}
