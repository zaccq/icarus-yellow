#!/bin/bash

BUILD_FLAGS=""

killall -9 -q JLinkRTTLogger

debug=0
pristine=0
term=0
flash=0
help=0
build=0

while getopts "pdtbfbh" flag
do
    case "${flag}" in
        p) pristine=1;;
        d) debug=1;;
        t) term=1;;
        f) flash=1;;
        b) build=1;;
        h)
            help=1
            echo "Project Icarus-Yellow mobile node build srcipt"
            echo "Usage: ./make.sh [flags]"
            echo "Flags:"
            echo "-p"
            echo "    pristine build (removes build directory)"
            echo "-b"
            echo "    build"
            echo "-f"
            echo "    flash after building (implied by -r or -d)"
            echo "-t"
            echo "    open rtt terminal after flashing (implies -f)"
            echo "-d"
            echo "    open debug session after flashing (implies -f)"
        ;;
    esac
done

if [ $help -ne 1 ]
then
    if [ $debug -eq 1 ] || [ $term -eq 1 ]
    then
        flash=1
    fi

    if [ $pristine -eq 1 ]
    then
        rm -rf build
        BUILD_FLAGS="${FLAGS} -p -c"
    fi

    if [ $build -eq 1 ]
    then
        west build ${BUILD_FLAGS}
    fi  && \
    if [ $flash -eq 1 ]
    then
        west flash
    fi && \
    if [ $debug -eq 1 ]
    then
        west debug
    elif [ $term -eq 1 ]
    then
        sleep 1 && \
        (JLinkRTTLogger -Device NRF52832_XXAA -RTTChannel 0 \ -if SWD -Speed 4000 ~/rtt.log &)
        sleep 2
        nc localhost 19021
    fi
fi