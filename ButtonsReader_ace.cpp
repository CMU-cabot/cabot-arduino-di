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

#include "ButtonsReader_ace.h"
//#define BTN1_PIN (2) // up
//#define BTN2_PIN (3) // down
//#define BTN3_PIN (4) // left
//#define BTN4_PIN (5) // right

ButtonsReader_ace::ButtonsReader_ace(ros::NodeHandle &nh, uart_com& cm):
  SensorReader(nh),
  cm(cm),
  b1_pub_("pushed_1", &b1_msg_),
  b2_pub_("pushed_2", &b2_msg_),
  b3_pub_("pushed_3", &b3_msg_),
  b4_pub_("pushed_4", &b4_msg_),
  b5_pub_("pushed_5", &b5_msg_)
{
  nh.advertise(b1_pub_);
  nh.advertise(b2_pub_);
  nh.advertise(b3_pub_);
  nh.advertise(b4_pub_);
  nh.advertise(b5_pub_);
}

void ButtonsReader_ace::init(){
  /*pinMode(b1_pin_, INPUT_PULLUP);
  pinMode(b2_pin_, INPUT_PULLUP);
  pinMode(b3_pin_, INPUT_PULLUP);
  pinMode(b4_pin_, INPUT_PULLUP);*/
}

void ButtonsReader_ace::update() {
  bool reading_1 = cm.switch_up;//!digitalRead(b1_pin_);
  bool reading_2 = cm.switch_down;//!digitalRead(b2_pin_);
  bool reading_3 = cm.switch_left;//!digitalRead(b3_pin_);
  bool reading_4 = cm.switch_right;//!digitalRead(b4_pin_);
  bool reading_5 = cm.switch_center;

  /*for(int i = 0; i < 10; i++) {
    delayMicroseconds(10);
    reading_1 = reading_1 && cm.switch_up;//!digitalRead(b1_pin_);
    reading_2 = reading_2 && cm.switch_down;//!digitalRead(b2_pin_);
    reading_3 = reading_3 && cm.switch_left;//!digitalRead(b3_pin_);  
    reading_4 = reading_4 && cm.switch_right;//!digitalRead(b4_pin_);
    reading_5 = reading_5 && cm.switch_center;
  }*/
  
  b1_msg_.data = reading_1;
  b2_msg_.data = reading_2;
  b3_msg_.data = reading_3;
  b4_msg_.data = reading_4;
  b5_msg_.data = reading_5;
        
  b1_pub_.publish(&b1_msg_);
  b2_pub_.publish(&b2_msg_);  
  b3_pub_.publish(&b3_msg_);
  b4_pub_.publish(&b4_msg_); 
  b5_pub_.publish(&b5_msg_);
}
