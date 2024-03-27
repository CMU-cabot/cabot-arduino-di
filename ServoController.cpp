/*******************************************************************************
 * Copyright (c) 2024  ALPS ALPINE CO., LTD.
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

#include "ServoController.hpp"

#define UART Serial1

const int8_t EN_PIN = 15;
const int8_t RX_PIN = 2;
const int8_t TX_PIN = 4;
const long BAUDRATE = 115200;
const int TIMEOUT = 200;
const int SERVO_DEV_ID = 0;
int16_t ServoController::KRS_position = 0;

IcsHardSerialClass krs(&UART, EN_PIN, BAUDRATE, TIMEOUT);

ServoController::ServoController(cabot::Handle &ch, uart_com &cm):
  SensorReader(ch), cm(cm)
{
  ch.subscribe(0x36, [](const int16_t msg) {servo_target_msg_(msg);});
  ch.subscribe(0x37, [](const bool msg) {servo_free_msg_(msg);});
}

int16_t ServoController::get_servo_direction(int servo_pos) {
  int16_t direction = (float)(servo_pos - 7500) * 180 / 5000;

  if (direction > 90) {
    direction = 90;
  } else if (direction < -90) {
    direction = -90;
  }

  return direction;
}

void ServoController::servo_target_msg_(int16_t direction) {
  // direction = Local_planの(終点姿勢角 - 始点姿勢角), +方向: 左回転, -方向: 右回転
  if (direction > 90) {
    direction = 90;
  } else if (direction < -90) {
    direction = -90;
  }

  int16_t servo_position = 7500 + (float)direction * 5000 / 180;
  krs.setPos(SERVO_DEV_ID, servo_position);                                      // 3500(左) ~ 7500(中央) ~ 11500(右)
}

void ServoController::servo_free_msg_(bool free_flag) {
  if (free_flag) {
    krs.setFree(SERVO_DEV_ID);
  }
}

void ServoController::init() {
  UART.setPins(RX_PIN, TX_PIN);
  krs.begin();

  for (int i = 0; i < 10; i++) {
    krs.setPos(SERVO_DEV_ID, 7500);
    delay(50);
  }

  krs.setFree(SERVO_DEV_ID);
}

void ServoController::update() {
  KRS_position = get_servo_direction(krs.getPos(SERVO_DEV_ID));
  ch_.publish(0x38, (int16_t)KRS_position);
}
