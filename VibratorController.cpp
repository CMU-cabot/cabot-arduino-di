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
 #include "VibratorController.hpp"

// keep the instance as static for callback
VibratorController * inst;

int ff2percent(int ff)
{
  return static_cast<double>((ff * 100) / 255.0);
}

VibratorController::VibratorController(cabot::Handle & ch, uart_com & cm)
: SensorReader(ch),
  cm(cm)
{
  inst = this;
  ch.subscribe(
    0x20, [](const uint8_t msg) {
      inst->ch_.loginfo("setting vibrator1");
      String buf = "setting vibfrator1,";
      buf += String(msg);
      buf += ",";
      buf += String(inst->cm.motor_r);
      buf += ",";
      buf += String(inst->cm.motor_c);
      buf += ",";
      buf += String(inst->cm.motor_l);

      inst->ch_.loginfo(const_cast<char*>(buf.c_str()));
      inst->cm.set_mot_c(ff2percent(msg));
    });
  ch.subscribe(0x21, [](const uint8_t msg) { /* nop: not supported */});
  ch.subscribe(0x22, [](const uint8_t msg) {inst->cm.set_mot_l(ff2percent(msg));});
  ch.subscribe(0x23, [](const uint8_t msg) {inst->cm.set_mot_r(ff2percent(msg));});
}

void VibratorController::init()
{
  ch_.loginfo("initializing vibrator controller");
  cm.set_mot(0, 0, 0);
}

void VibratorController::update() {}
