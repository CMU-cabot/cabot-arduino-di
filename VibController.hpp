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

#ifndef ARDUINO_NODE_VIB_CONTROLLER_H
#define ARDUINO_NODE_VIB_CONTROLLER_H

#include <Haptic_Driver.h>
#include <Wire.h>
#include "SensorReader.h"
#include "uart_com.h"

class VibController: public SensorReader {
  static uint8_t MAX_AMP;
  static uint16_t MAX_FREQ;
  static uint8_t vib_power_;
  static uint16_t vib_freq_;
  static bool is_enabled_;
  uart_com &cm;

public:
  VibController(cabot::Handle &ch, uart_com &cm);
  void init();
  void update();
  static void vib_msg_(bool msg);
  static void vib_power_msg_(uint8_t msg);
  static void vib_freq_msg_(uint16_t msg);
  static void setHapticParams(uint8_t amp, uint16_t freq, bool is_enabled);
};

#endif //ARDUINO_NODE_VIB_CONTROLLER_H
