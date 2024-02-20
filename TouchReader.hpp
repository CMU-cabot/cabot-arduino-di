/*******************************************************************************
 * Copyright (c) 2020, 2023  Carnegie Mellon University, IBM Corporation, and others
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

#ifndef TOUCHREADER_HPP_
#define TOUCHREADER_HPP_

#include <Wire.h>
#include <Adafruit_MPR121.h>
#include "SensorReader.h"
#include "uart_com.h"  // NOLINT

class TouchReader: public SensorReader {
  Adafruit_MPR121 cap_;
  int16_t touched_;
  uart_com & cm;
  bool is_continuous_;
  uint8_t diag_status_;
  String diag_message_;
  int count_;

public:
  TouchReader(cabot::Handle & ch, uart_com & cm);
  void init();
  void init(
    uint8_t touch_baseline, uint8_t touch_threshold,
    uint8_t release_threshold);
  void update();
  void diag_pub();
  void check_touch_raw(int16_t touch_raw);

private:
  void set_mode(uint8_t touch_baseline);
};

#endif  // TOUCHREADER_HPP_
