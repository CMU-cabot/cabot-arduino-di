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

#ifndef ARDUINO_NODE_SERVO_CONTROLLER_H
#define ARDUINO_NODE_SERVO_CONTROLLER_H

#include <IcsHardSerialClass.h>
#include <Wire.h>
#include "SensorReader.h"
#include "uart_com.h"  // NOLINT

class ServoController: public SensorReader {
  static int16_t KRS_position;
  uart_com &cm;

public:
  ServoController(cabot::Handle &ch, uart_com &cm);
  void init();
  void update();
  static int16_t get_servo_direction(int servo_pos);
  static void servo_target_msg_(int16_t msg);
  static void servo_free_msg_(bool msg);
};

#endif //ARDUINO_NODE_SERVO_CONTROLLER_H
