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

#ifndef ARDUINO_NODE_BUTTONS_READER_ACE_H
#define ARDUINO_NODE_BUTTONS_READER_ACE_H

#include <Wire.h>
#include <std_msgs/Bool.h>
#include "SensorReader.h"
#include "uart_com.h"

class ButtonsReader_ace: public SensorReader {
  ros::Publisher b1_pub_;
  ros::Publisher b2_pub_;
  ros::Publisher b3_pub_;
  ros::Publisher b4_pub_;
  ros::Publisher b5_pub_;
  std_msgs::Bool b1_msg_;
  std_msgs::Bool b2_msg_;
  std_msgs::Bool b3_msg_;
  std_msgs::Bool b4_msg_;
  std_msgs::Bool b5_msg_;
  uart_com& cm;
public:
  ButtonsReader_ace(ros::NodeHandle &nh, uart_com& cm);
  void init();
  void update();
};

#endif //ARDUINO_NODE_BUTTONS_READER_ACE_H
