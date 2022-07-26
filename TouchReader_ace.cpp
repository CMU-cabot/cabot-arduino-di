/*******************************************************************************
 * Copyright (c) 2020, 2022 Carnegie Mellon University, IBM Corporation, and others
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

#include "TouchReader_ace.h"

TouchReader_ace::TouchReader_ace(ros::NodeHandle &nh, uart_com& cm):
  SensorReader(nh),
  cm(cm),
  touch_pub_("touch", &touch_msg_),
  raw_pub_("touch_raw", &raw_msg_)/*,
  vel_pub_("touch_speed", &vel_msg_)*/
{
  nh.advertise(touch_pub_);
  nh.advertise(raw_pub_);
  //nh.advertise(vel_pub_);
}

void TouchReader_ace::init() {
  //TODO
  initialized_ = true;
}

void TouchReader_ace::init(uint8_t touch_baseline, uint8_t touch_threshold, uint8_t release_threshold) {
  //cm.set_thresh(touch_threshold);
  nh_.loginfo("Touch initialized");
  initialized_ = true;
}

void TouchReader_ace::set_mode(uint8_t touch_baseline) {
  nh_.loginfo("Touch ready");
}

void TouchReader_ace::update() {
  if (!initialized_) {
    return;
  }
  int touched = cm.touch ? 1 : 0;
  touch_msg_.data = touched;
  touch_pub_.publish( &touch_msg_ );
  
  raw_msg_.data = cm.capacitance;//TBR
  raw_pub_.publish( &raw_msg_ );
  
  //vel_msg_.data = (touched & 0x01) ? 2.0 : 0;
  //vel_pub_.publish( &vel_msg_ );
}
