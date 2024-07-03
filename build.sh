#!/bin/bash
function err {
    >&2 red "[ERROR] "$@
}
function red {
    echo -en "\033[31m"  ## red
    echo $@
    echo -en "\033[0m"  ## reset color
}
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
    echo "-m <mode>  set mode (I1/M1/M2) **REQUIRED** to set"
}

: ${ARDUINO_BOARD:="esp32:esp32:esp32"}
: ${ARDUINO_PORT:="/dev/ttyESP32"}
: ${ARDUINO_MODE:=} # mode should be specified

board=$ARDUINO_BOARD
port=$ARDUINO_PORT
mode=$ARDUINO_MODE

while getopts "hb:p:m:" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	b)
	    board=$OPTARG
	    ;;
	p)
	    port=$OPTARG
	    ;;
    m)
        mode=$OPTARG
        ;;
    esac
done
shift $((OPTIND-1))

target=$1
if [ -z $target ]; then
    target=build
fi

function build() {
    echo "building for $board, $mode..."
    echo "arduino-cli compile -b $board --build-property compiler.cpp.extra_flags=-D$mode ."
    arduino-cli compile -b $board --build-property compiler.cpp.extra_flags=-D$mode .
    if [ $? -ne 0 ]; then
        err "Please check mode ($mode)"
    fi
}

function upload() {
    echo "uploading for $board at $port..."
    arduino-cli upload -b $board -p $port .
    while [ $? -ne 0 ]; do
	arduino-cli upload -b $board -p $port .
    done	
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
