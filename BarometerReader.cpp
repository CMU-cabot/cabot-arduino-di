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

#include "BarometerReader.hpp"  // NOLINT

BarometerReader::BarometerReader(cabot::Handle & ch)
: SensorReader(ch) {}

void BarometerReader::init()
{
  Wire.begin(21, 22);
  if (!bme_.begin(0x77, &Wire)) {
    ch_.loginfo("Ooops, no BME280 detected ... Check your wiring or I2C ADDR!");
    return;
  }
  initialized_ = true;

  bme_.setSampling(
    Adafruit_BME280::MODE_NORMAL,                    /* Operating Mode. */
    Adafruit_BME280::SAMPLING_X2,                    /* Temp. oversampling */
    Adafruit_BME280::SAMPLING_X16,                   /* Pressure oversampling */
    Adafruit_BME280::SAMPLING_X16,                   /* Humidity oversampling */
    Adafruit_BME280::FILTER_X16,                     /* Filtering. */
    Adafruit_BME280::STANDBY_MS_500);                /* Standby time. */
}

void BarometerReader::update()
{
  if (!initialized_) {
    return;
  }

  ch_.publish(0x15, bme_.readPressure());
  ch_.publish(0x16, bme_.readTemperature());
}
