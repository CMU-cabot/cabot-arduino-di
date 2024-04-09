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

#include "VibController.hpp"

uint8_t VibController::MAX_AMP = 127;
uint16_t VibController::MAX_FREQ = 1000;
uint8_t VibController::vib_power_ = 0;
uint16_t VibController::vib_freq_ = 0;
bool VibController::is_enabled_ = false;

Haptic_Driver hapticDriver;
hapticSettings defaultHapticSettings;

VibController::VibController(cabot::Handle &ch, uart_com &cm)
: SensorReader(ch), cm(cm)
{
  ch.subscribe(0x31, [](const bool msg) {vib_msg_(msg);});
  ch.subscribe(0x32, [](const uint8_t msg) {vib_power_msg_(msg);});
  ch.subscribe(0x33, [](const uint16_t msg) {vib_freq_msg_(msg);});
}

void VibController::vib_msg_(bool state) {
  is_enabled_ = state;
  setHapticParams(vib_power_, vib_freq_, is_enabled_);
}

void VibController::vib_power_msg_(uint8_t power) {
  vib_power_ = power;
  if (vib_power_ > MAX_AMP) {
    vib_power_ = MAX_AMP;
  }
  setHapticParams(vib_power_, vib_freq_, is_enabled_);
}

void VibController::vib_freq_msg_(uint16_t freq) {
  vib_freq_ = freq;
  if (vib_freq_ > MAX_FREQ) { 
    vib_freq_ = MAX_FREQ;
  }
  setHapticParams(vib_power_, vib_freq_, is_enabled_);
}

void VibController::setHapticParams(uint8_t amp, uint16_t freq, bool is_enabled) {
  if (is_enabled) {
    hapticDriver.setActuatorLRAfreq(freq);
    hapticDriver.setVibratePower(amp);
  } else {
    hapticDriver.setVibratePower(0);
  }
}

void VibController::init() {
  Wire.begin();
  Wire.setClock(1000000);
  hapticDriver.begin();

  defaultHapticSettings.motorType = LRA_TYPE;
  defaultHapticSettings.nomVolt = 2.0;          // Nominal Voltage: 2.0V
  defaultHapticSettings.absVolt = 2.0;          // Absolute Voltage: 2.0V
  defaultHapticSettings.currMax = 250;          // Maximum Current: 250 mA
  defaultHapticSettings.impedance = 8;          // Channel Impedance: 8 Ohms
  defaultHapticSettings.lraFreq = 166;          // LRA Frequency: 166 Hz
  while (!hapticDriver.setMotor(defaultHapticSettings)) {
    delay(100);
  }

  hapticDriver.setOperationMode(DRO_MODE);
  hapticDriver.enableEmbeddedOperation(true);
  hapticDriver.enableFreqTrack(false);
  hapticDriver.enableAcceleration(true);
  hapticDriver.enableRapidStop(true);
  hapticDriver.enableV2iFactorFreeze(true);
  hapticDriver.enableDoubleRange(false);
  delay(100);
}

void VibController::update() {
  ch_.publish(0x34, (uint8_t)vib_power_);
  ch_.publish(0x35, (uint16_t)vib_freq_);
}
