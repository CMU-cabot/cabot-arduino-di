/*******************************************************************************
 * Copyright (c) 2020, 2023  Carnegie Mellon University, IBM Corporation, and others
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

#ifndef VIBRATORCONTROLLER_HPP_
#define VIBRATORCONTROLLER_HPP_

#include <Wire.h>
#include "SensorReader.h"
#ifdef ESP32
#include <analogWrite.h>
#endif
#include "uart_com.h"  // NOLINT

class VibratorController: public SensorReader {
  uart_com & cm;
  uint16_t vibrations[4];
public:
  VibratorController(cabot::Handle & ch, uart_com & cm);
  void init() override;
  void update() override;
};

#endif  // VIBRATORCONTROLLER_HPP_
