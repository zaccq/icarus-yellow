#!/bin/bash

BUILD_FLAGS=""

SCRIPTS_DIR="../../scripts"

pristine=0
flash=0
help=0
term=0
build=0
serial="/dev/ttyACM0"
outfile="build.zip"

while getopts "s:t:pfbh" flag
do
    case "${flag}" in
        s) serial="${OPTARG}";;
        t) term="$OPTARG";;
        p) pristine=1;;
        f) flash=1;;
        b) build=1;;
        *)
            help=1
            echo "Project Icarus-Yellow mobile node build script"
            echo "Usage: ./make.sh [flags] [arguments]"
            echo "Flags:"
            echo "-p"
            echo "    pristine build (removes build directory)"
            echo "-b"
            echo "    build"
            echo "-f"
            echo "    flash after building (implied by -t)"
            echo "Arguments:"
            echo "-s [\"tty\"]"
            echo "    serial tty device to use (default: /dev/ttyACM0)"
            echo "-t <\"screen\"|\"python\">"
            echo "    open a terminal after flashing"
        ;;
    esac
done

if [ $help -ne 1 ]
then

    read -n 1 -s -r -p "Put the dongle in dfu mode then press any key to continue"

    if [ -n "$term" ]
    then
        flash=1
    fi

    if [ $pristine -eq 1 ]
    then
        rm -rf build
        BUILD_FLAGS="${BUILD_FLAGS}-p -c"
    fi
    
    if [ $build -eq 1 ]
    then
        west build ${BUILD_FLAGS}
    fi  && \
    if [ $flash -eq 1 ]
    then
        nrfutil pkg generate --hw-version 52 --sd-req=0x00 \
        --application build/zephyr/zephyr.hex \
        --application-version 1 "${outfile}" && \
        nrfutil -v dfu usb-serial -pkg "${outfile}" -p "${serial}" -cd 1
    fi && \
    if [ "$term" = "screen" ]
    then
        sleep 2
        screen ${serial}
    elif [ "$term" = "python" ]
    then
        sleep 2
        python "${SCRIPTS_DIR}/serial_logger.py" -t "${serial}"
    fi
fi