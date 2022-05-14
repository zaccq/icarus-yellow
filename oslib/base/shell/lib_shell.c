#include "lib_shell.h"

const struct device *shell_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));