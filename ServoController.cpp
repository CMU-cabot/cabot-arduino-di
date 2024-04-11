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

// keep the instance as static for callback
ServoController * servo_inst;

ServoController::ServoController(cabot::Handle &ch, uart_com &cm)
: SensorReader(ch), 
  cm(cm)
{
  servo_inst = this;
  ch.subscribe(0x36, [](const int16_t msg) {servo_inst->cm.set_servo_pos(msg);});
  ch.subscribe(0x37, [](const bool msg) {servo_inst->cm.set_servo_free(msg);});
}

void ServoController::init() {
  ch_.loginfo("initializing servo controller");
  cm.set_servo_pos(0);
}

void ServoController::update() {
    ch_.publish(0x38, (int16_t)cm.servo_position);
}
