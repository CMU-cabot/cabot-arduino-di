/*******************************************************************************
 * Copyright (c) 2020, 2023  Carnegie Mellon University and Miraikan
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

#include "ButtonsReader.hpp"  // NOLINT

ButtonsReader::ButtonsReader(cabot::Handle & ch, uart_com & cm)
: SensorReader(ch), cm(cm) {}

void ButtonsReader::init() {}

void ButtonsReader::update()
{
  bool reading_1 = cm.switch_up;
  bool reading_2 = cm.switch_down;
  bool reading_3 = cm.switch_left;
  bool reading_4 = cm.switch_right;
  bool reading_5 = cm.switch_center;


  int8_t temp = reading_1 ? 0x01 : 0x00 | reading_2 ? 0x02 : 0x00 | reading_3 ? 0x04 : 0x00 |
    reading_4 ? 0x08 : 0x00 | reading_5 ? 0x10 : 0x00;

  ch_.publish(0x12, temp);
}
