/*******************************************************************************
 * Copyright (c) 2020  Carnegie Mellon University
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

#include "BarometerReader.h"

BarometerReader::BarometerReader(ros::NodeHandle &nh):
  SensorReader(nh),
  fp_pub_("pressure", &fp_msg_),
  tmp_pub_("temperature", &tmp_msg_)
{
  nh_.advertise(fp_pub_);
  nh_.advertise(tmp_pub_);
}

void BarometerReader::init(){
  Wire.begin(21,22);
  if(!bmp_.begin(0x76, &Wire))
  {
    nh_.loginfo("Ooops, no BME280 detected ... Check your wiring or I2C ADDR!");
    return;
  }
  initialized_ = true;
  
  bmp_.setSampling(Adafruit_BME280::MODE_NORMAL,     /* Operating Mode. */
		   Adafruit_BME280::SAMPLING_X2,     /* Temp. oversampling */
		   Adafruit_BME280::SAMPLING_X16,    /* Pressure oversampling */
       Adafruit_BME280::SAMPLING_X16, 
		   Adafruit_BME280::FILTER_X16,      /* Filtering. */
		   Adafruit_BME280::STANDBY_MS_500); /* Standby time. */
}

void BarometerReader::update(){
  if (!initialized_) {
    return;
  }
  fp_msg_.fluid_pressure = bmp_.readPressure();
  fp_msg_.variance = 0;
  fp_msg_.header.stamp = nh_.now();
  fp_msg_.header.frame_id = "bmp_frame";
  fp_pub_.publish( &fp_msg_ );
  
  tmp_msg_.temperature = bmp_.readTemperature();
  tmp_msg_.variance = 0;
  tmp_msg_.header.stamp = nh_.now();
  tmp_msg_.header.frame_id = "bmp_frame";
  tmp_pub_.publish( &tmp_msg_ );
}
