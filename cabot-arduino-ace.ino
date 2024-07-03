/*******************************************************************************
 * Copyright (c) 2020, 2024  Carnegie Mellon University and Miraikan
 * Copyright (c) 2024  ALPS ALPINE CO.,LTD.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#ifdef ESP32
#undef ESP32
#include "CaBotHandle.hpp"
#define ESP32
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#else
#include "CaBotHandle.hpp"
#endif

#include <arduino-timer.h>
#include "Arduino.h"

#include "uart_com.h"
#include "BarometerReader.hpp"
#include "ButtonsReader.hpp"
// #include "Heartbeat.h"
#ifdef I1
#include "IMUReaderI1.hpp"
#else
#include "IMUReaderM.hpp"
#endif

#include "WiFiReader.hpp"
#include "TouchReader.hpp"
#include "VibratorController.hpp"
#include "ServoController.hpp"

cabot::Handle ch;
Timer < 10 > timer;

// configurations
#define BAUDRATE (115200UL)

#define HEARTBEAT_DELAY (20)

#ifdef ESP32
// TODO: need to reconfigure
#define BTN1_PIN (13) // up
#define BTN2_PIN (14) // down
#define BTN3_PIN (15) // left
#define BTN4_PIN (16) // right

#define VIB1_PIN (19)  //front
#define VIB2_PIN (20)   //back //not using
#define VIB3_PIN (18)  //left
#define VIB4_PIN (17)   //right
#else
#define BTN1_PIN (2) // up
#define BTN2_PIN (3) // down
#define BTN3_PIN (4) // left
#define BTN4_PIN (5) // right

#define VIB1_PIN (11)  //front
#define VIB2_PIN (6)   //back //not using
#define VIB3_PIN (10)  //left
#define VIB4_PIN (9)   //right
#endif


#define TOUCH_BASELINE (128)
#define TOUCH_THRESHOLD_DEFAULT (64)
#define RELEASE_THRESHOLD_DEFAULT (24)

#define TIMEOUT_DEFAULT (500)
uart_com urt_cm(ch);

// sensors
BarometerReader bmpReader(ch);
ButtonsReader buttonsReader(ch, urt_cm);
#ifdef I1
IMUReaderI1 imuReader(ch);
#else
IMUReaderM imuReader(ch);
#endif
WiFiReader wifiReader(ch);
TouchReader touchReader(ch, urt_cm);

// controllers
// Heartbeat heartbeat(LED_BUILTIN, HEARTBEAT_DELAY);
ServoController servoController(ch, urt_cm);
VibratorController vibratorController(ch, urt_cm);

void setup()
{
  // set baud rate
  ch.setBaudRate(BAUDRATE);
  urt_cm.begin(115200);

  // connect to rosserial
  ch.init();

  ch.loginfo("Connected");
  while (!ch.connected()) {
    ch.spinOnce();
  }

  int run_imu_calibration = 0;
  ch.getParam("run_imu_calibration", &run_imu_calibration, 1, TIMEOUT_DEFAULT);
  if (run_imu_calibration != 0) {
    imuReader.calibration();
    timer.every(
      100, [] (void *) {
      imuReader.update();
      imuReader.update_calibration();
      return true;
    });
    ch.loginfo("Calibration Mode started");
    return;
  }

  int calibration_params[22];
  uint8_t * offsets = NULL;
  if (ch.getParam(
      "calibration_params", calibration_params, 22,
      TIMEOUT_DEFAULT))
  {
    offsets = (uint8_t *)malloc(sizeof(uint8_t) * 22);
    for (int i = 0; i < 22; i++) {
      offsets[i] = calibration_params[i] & 0xFF;
    }
  } else {
    ch.logwarn("clibration_params is needed to use IMU (BNO055) correctly.");
    ch.logwarn("You can run calibration by setting _run_imu_calibration:=1");
    ch.logwarn("You can check calibration value with /calibration topic.");
    ch.logwarn(
      "First 22 byte is calibration data, following 4 byte is calibration "
      "status for");
    ch.logwarn(
      "System, Gyro, Accel, Magnet, 0 (not configured) <-> 3 (configured)");
    ch.logwarn("Specify like calibration_params:=[0, 0, 0, 0 ...]");
    ch.logwarn("Visit the following link to check how to calibrate sensoe");
    ch.logwarn(
      "https://learn.adafruit.com/"
      "adafruit-bno055-absolute-orientation-sensor/device-calibration");
  }

  ch.loginfo("setting up WiFi");
  wifiReader.init(
    [] (char * buf) {
    // ch.loginfo(buf); //TODO*
  });


  int touch_params[3];
  int touch_baseline;
  int touch_threshold;
  int release_threshold;
  if (!ch.getParam("touch_params", touch_params, 3, TIMEOUT_DEFAULT)) {
    ch.logwarn(
      "Please use touch_params:=[baseline,touch,release] format to set touch "
      "params");
    touch_baseline = TOUCH_BASELINE;
    touch_threshold = TOUCH_THRESHOLD_DEFAULT;
    release_threshold = RELEASE_THRESHOLD_DEFAULT;
    ch.logwarn(
      " touched  if the raw value is less   than touch_params[0] - "
      "touch_params[1]");
    ch.logwarn(
      " released if the raw value is higher than touch_params[0] - "
      "touch_params[2]");
  } else {
    touch_baseline = touch_params[0];
    touch_threshold = touch_params[1];
    release_threshold = touch_params[2];
  }
  char default_values[48];
  snprintf(
    default_values, 48, "Using [%d, %d, %d] for touch_params",
    touch_baseline, touch_threshold, release_threshold);
  ch.loginfo(default_values);

  // initialize
  ch.loginfo("starting uart com");
  urt_cm.start();
  ch.loginfo("setting up BMP280");
  bmpReader.init();
  ch.loginfo("setting up Buttons");
  buttonsReader.init();
  ch.loginfo("setting up BNO055");
  imuReader.init(offsets);
  ch.loginfo("setting up MPR121");
  touchReader.init(touch_baseline, touch_threshold, release_threshold);
  ch.loginfo("setting up vibrations");
  vibratorController.init();
  // ch.loginfo("setting up heartbeat");
  // heartbeat.init();
  ch.loginfo("setting up servoMotor");
  servoController.init();

  // wait sensors ready
  delay(100);

  // set timers
  timer.every(
    1000, [] (void *) {
    ch.sync();
    return true;
  });

  timer.every(
    500, [] (void *) {
    bmpReader.update();
    urt_cm.publish();
    touchReader.diag_pub();
    return true;
  });

  timer.every(
    20, [] (void *) {
    // heartbeat.update();
    buttonsReader.update();
    touchReader.update();
    servoController.update();
    return true;
  });

  timer.every(
    10, [] (void *) {
    imuReader.update();
    vibratorController.update();
    return true;
  });

  timer.every(
    20, [] (void *) {
    wifiReader.update();
    return true;
  });

  ch.loginfo("Arduino is ready");
}

void loop()
{
  timer.tick < void > ();
  urt_cm.update();
  ch.spinOnce();
}

void restart()
{
  ESP.restart();
}
