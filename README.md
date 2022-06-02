# icarus-yellow
Bluetooth and Machine Learning based Activity Classification Project

## References

This project uses snippets of code from the following zephyr samples:

- `basics/blinky`
- `bluetooth/central_hr`

This project also uses code from the Sparkfun MPU9250 DMP library available [here][1].

## Dependencies

Project Icarus-Yellow has the following software dependencies:
- west
- cmake
- python 3.8.x
- zephyr sdk
- bash

## Hardware
Project Icarus-Yellow uses the following hardware:
- thingy:52 IOT development kit
- nrf52840
- JLink programmer

## Build Instructions

This project was devloped in linux and assumes a linux/unix-like environment, all versions of Windows are unsupported at this time.

### Dongle

Assuming all dependencies are ment, the code for the dongle may be compiled and flashed by running `./make.sh -p -b -f` in the `apps/base` directory and following any prompts. If prompted to put the device into bootloader mode, press the horizontal-facing button furthest from the usb port. A red led on the board should start flashing slowly if this is successful.

### Wearable

The code for the Thingy:52 may be built and flashed by running `./make.sh -p -b -f` in the `apps/mobile` directory and assumes that the device is powered on and plugged into the JLink programmer.

## Running Instructions

The PC software communicates with a tago.io instance via HTTPS and the dongle via serial over USB. For it to work correctly the dongle and Thingy:52 should be programmed as above. The dongle should be connected to any available USB port, and the Thingy:52 should be charged and powered on. Once that is done the pc software may be run.

The pc software is written in python, and may be executed by running `python main.py -t /dev/ttyACM0 -p "CSSE4011:~\$"` substituting the tty device that the dongle enumerates to for `/dev/ttyACM0` which is where the device tries to enumerate on linux. Mac users will need to identify the terminal by running `ls /dev/tty0.*`.

## Extensibility

With small changes to the mobile node code to have unique UUIDs, the system may be extended to have multiple mobile nodes active concurrently. As the bluetooth protocol in use is GATT, the dongle is not strictly necessary, and any programmable BLE capable device could be used instead to implement the same functionality. If the interface was changed from connection-based to scan-advertise based this would trivially allow multiple nodes to co-exist.

The pc software is very simple but could be extended to multiple nodes by giving each node its own thread and classifier. The classification dataset is very easy to update only requiring some logged readings which can be taken by running main.py with the appropriate flags. The data that then needs to be extracted for this model is the absolute mean and standard-deviation of all three axes of the accelerometer, gyroscope, acceleration vector magnitude, and angular velocity vector magnitude. The absolute means for the gyroscope axes and the vector magnitude were not used in this model as they were always very close to (0,0,0) and 1 respectively but may be useful for other activities.

[1]: <https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/>
