#!/bin/bash

function help() {
    echo "$0 [options] [build|upload|all]"
    echo ""
    echo "build      build the code (default action)"
    echo "upload     upload the code"
    echo "all        build and upload if build is success"
    echo ""
    echo "-h         show this help"
    echo "-b         set board (default=esp32:esp32:esp32)"
    echo "-p         set port (default=/dev/ttyESP32)"
}

: ${ARDUINO_BOARD:="esp32:esp32:esp32"}
: ${ARDUINO_PORT:="/dev/ttyESP32"}

board=$ARDUINO_BOARD
port=$ARDUINO_PORT

while getopts "hb:p:" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	b)
	    borad=$OPTARG
	    ;;
	p)
	    port=$OPTARG
	    ;;
    esac
done

target=$1
if [ -z $target ]; then
    target=build
fi

function build() {
    echo "building..."
    arduino-cli compile -b $board .
}

function upload() {
    echo "uploading..."
    arduino-cli upload -b $board -p $port .
}    

if [ $target == "build" ]; then
    build
fi

if [ $target == "upload" ]; then
    upload
fi

if [ $target == "all" ]; then
    build && upload
fi
