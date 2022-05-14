cmake_minimum_required(VERSION 3.20.0)

set(BOARD thingy52_nrf52832)
set(BOARD_FLASH_RUNNER jlink)

set(OSLIB_DIR ../../oslib/)
set(EXT_LIB_DIR ../../extlib/)
set(MOBILE_NODE_DRIVER_LIB_DIR ${OSLIB_DIR}/mobile)
set(BLE_DRIVER_LIB_DIR ${MOBILE_NODE_DRIVER_LIB_DIR}/ble)
set(SENSOR_DRIVER_LIB_DIR ${MOBILE_NODE_DRIVER_LIB_DIR}/sensors)
set(IO_DRIVER_LIB_DIR ${MOBILE_NODE_DRIVER_LIB_DIR}/io)
set(MPU9250_LIB_DIR ${EXT_LIB_DIR}/mpu9250)


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
  -Wunused-function
  -pipe
  -ggdb3
  -Og
  -DMPU9250
  -DAK8963_SECONDARY
  -DCOMPASS_ENABLED
)

set(CONF_FILE prj.conf)

set(DTC_OVERLAY_FILE
    speaker.overlay
    ${SENSOR_DRIVER_LIB_DIR}/mpu9250.overlay
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(icarus-yellow-mobile)

include_directories(
    ${SENSOR_DRIVER_LIB_DIR}
    ${BLE_DRIVER_LIB_DIR}
    ${IO_DRIVER_LIB_DIR}
    ${MPU9250_LIB_DIR}
)

target_sources(app PRIVATE
    src/main.c
    ${SENSOR_DRIVER_LIB_DIR}/temp_humidity_sensor.c
    ${SENSOR_DRIVER_LIB_DIR}/motion_sensor.c
    ${SENSOR_DRIVER_LIB_DIR}/air_quality_sensor.c
    ${SENSOR_DRIVER_LIB_DIR}/temp_pressure_sensor.c
    ${SENSOR_DRIVER_LIB_DIR}/lib_mpu9250.c
    ${SENSOR_DRIVER_LIB_DIR}/tsk_mpu9250.c
    ${BLE_DRIVER_LIB_DIR}/scu_ble.c
    ${IO_DRIVER_LIB_DIR}/button.c
    ${IO_DRIVER_LIB_DIR}/led.c
    #${IO_DRIVER_LIB_DIR}/buzzer.c
    ${MPU9250_LIB_DIR}/inv_mpu_dmp_motion_driver.c
    ${MPU9250_LIB_DIR}/inv_mpu.c
)