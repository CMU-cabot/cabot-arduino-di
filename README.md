[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# CaBot Arduino DI (Directional Indicator)

# Usage
- see [cabot-drivers](https://github.com/CMU-cabot/cabot-drivers) to run the example
```
cabot-drivers$ docker compose run --rm driver bash
$ source install/setup.bash
$ ros2 run cabot_serial cabot_serial_node --ros-args -p port:=/dev/ttyESP32 [other parameters]
```

## parameters

- **run_imu_calibration** : `bool` Run IMU calibration if this flag is set
- **calibration_params** : `int[22]`
  - BNO055 calibration parameter. Follow instruction when you not specify this parameter.
- **touch_params** : [`touch_base (int)`, `touch_threshold (int)`, `release_threshold (int)`]
  - Touch threshold parameters
  - `touch_base` - base value when you don't touch the touch sensor
  - `touch_threshold` - if the value is below `touch_base - touch_threshold` then changes to touch state
  - `release_threshold` - After transitioning to touch state, the value bigger than `touch_base - release_threshold` then changes to release state


## Serial Connection

- It used to use rosserial for serial connection with ROS1
- ROS2 offers micro ROS, but it does not fit to our board, so we implemented custom serial communication
  - see CaBotHandle.cpp/hpp for the details

## Pre-requisites

### Hardware

One example of hardware components

- [ESP32](https://www.espressif.com/en/products/devkits)
- [MPR121](https://www.adafruit.com/product/1982) capacitive touch sensor
- [BNG055](https://www.adafruit.com/product/2472) 9-axis IMU
- [BME280](https://www.adafruit.com/product/2652) Barometric Pressure & Altitude Sensor
- 4/5 [push buttons](https://www.adafruit.com/product/4183)
- 3 [vibrators](https://www.sparkfun.com/products/17590)
- 1 [servo motor](https://kondo-robot.com/product/krs-3304r2-ics)
- Wires and headers

### Software (docker, arduino-ide)

```
  host $ docker-compose build
  host $ docker-compose run arduino
docker $ ./build.sh all                # build and upload (with -b esp32:esp32:esp32 -p /dev/ttyESP32)
                                       # you can set board by ARDUINO_BOARD, and port by ARDUINO_PORT environment variables

or use arduino-cli
docker $ arduino-cli compile -b esp32:esp32:esp32 .
docker $ arduino-cli upload -b esp32:esp32:esp32 -p /dev/ttyESP32 .
```
- change `-b <board type> -p <port>` for your environment
- linux is required to upload built sketch (Windows/Mac docker container does not support)


## Components description

- The Cabot's handle has three main features namely, touch sensor, push buttons (four of them), and vibrators (three of them).
- The touch sensor is used for giving user feedback to the robot while it is moving, to ensure that the robot moves along with the user without getting lost.
- The three vibrators are useful for giving alerts to the user in response to obstacles encountered in the front and on the sides of the robot while it is moving. They also provide alerts/ warnings to the user about upcoming left turns and right turns.
- The four push butons are useful for giving manual input signals to the robot, to command it to move in preferred directions (forward, backward, left turn, and right turn).
- The touch sensor is usually connected to a conductive material like copper film, or copper plate.
