cmake_minimum_required(VERSION 3.20.0)

set(BOARD nrf52840dongle_nrf52840)

set(OSLIB_DIR ../../oslib/)
set(EXT_LIB_DIR ../../extlib/)
set(LED_DRIVER_LIB_DIR ${OSLIB_DIR}/led)
set(BASE_NODE_DRIVER_LIB_DIR ${OSLIB_DIR}/base)
set(SHELL_LIB_DIR ${BASE_NODE_DRIVER_LIB_DIR}/shell)
set(BLE_DRIVER_LIB_DIR ${BASE_NODE_DRIVER_LIB_DIR}/ble)
set(HCI_LIB_DIR ${BASE_NODE_DRIVER_LIB_DIR}/hci)

add_compile_options(
  -Wall
  -Wextra
  -Wno-discarded-qualifiers
  -Wno-sign-compare
  -Wno-unused-variable
  -Wno-unused-parameter
  -Wno-cast-function-type
  -Wno-missing-field-initializers
  -Wno-type-limits
  -pipe
  -ggdb3
  -Og
)

set(CONF_FILE
    prj.conf
    ${OSLIB_DIR}/usb/usb.conf
    ${SHELL_LIB_DIR}/shell.conf
    ${BLE_DRIVER_LIB_DIR}/ble.conf
)

set(DTC_OVERLAY_FILE
    ${SHELL_LIB_DIR}/shell.overlay
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(icarus-yellow-base)

#Add Include Directories
include_directories(
    ${LED_DRIVER_LIB_DIR}
    ${SHELL_LIB_DIR}
    ${BLE_DRIVER_LIB_DIR}
    ${HCI_LIB_DIR}
    ${MPU9250_LIB_DIR}
)

#Add Target Sources
target_sources(app PRIVATE
    src/main.c
    ${LED_DRIVER_LIB_DIR}/lib_led.c
    ${LED_DRIVER_LIB_DIR}/tsk_led.c
    ${SHELL_LIB_DIR}/ahu_led_cmd.c
    ${SHELL_LIB_DIR}/ahu_time_cmd.c
    ${SHELL_LIB_DIR}/ahu_ble_hts221_cmd.c
    ${SHELL_LIB_DIR}/ahu_ble_lis2dh_cmd.c
    ${SHELL_LIB_DIR}/ahu_ble_lps22_cmd.c
    ${SHELL_LIB_DIR}/ahu_ble_ccs811_cmd.c
    ${SHELL_LIB_DIR}/ahu_ble_pushbutton_cmd.c
    ${SHELL_LIB_DIR}/ahu_ble_buzzer_cmd.c
    ${SHELL_LIB_DIR}/ahu_ble_rgb_cmd.c
    ${SHELL_LIB_DIR}/ahu_ble_dc_cmd.c
    ${SHELL_LIB_DIR}/ahu_ble_sample_cmd.c
    ${SHELL_LIB_DIR}/ahu_ble_all_cmd.c
    ${SHELL_LIB_DIR}/lib_shell.c
    ${HCI_LIB_DIR}/lib_hci.c
    ${BLE_DRIVER_LIB_DIR}/tsk_ble.c
    #${MPU9250_LIB_DIR}/inv_mpu_dmp_motion_driver.c
    #${MPU9250_LIB_DIR}/inv_mpu.c
)