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
 #include "VibratorController_ace.h"

//#define VIB1_PIN (19)  //front
//#define VIB2_PIN (20)   //back //not using
//#define VIB3_PIN (18)  //left
//#define VIB4_PIN (17)   //right

// keep the instance as static for callback
VibratorController_ace *inst;

int ff2percent(int ff){
  return (int)((double)(ff * 100) / 255.0);
}

VibratorController_ace::VibratorController_ace(ros::NodeHandle &nh, uart_com& cm):
  SensorReader(nh),
  cm(cm),
  vib1_sub_("vibrator1", [](const std_msgs::UInt8& msg) {
    inst->nh_.loginfo("setting vibrator1");
    String buf="setting vibfrator1,";
    buf += String(msg.data);
    buf += ",";
    buf += String(inst->cm.motor_r);
    buf += ",";
    buf += String(inst->cm.motor_c);
    buf += ",";
    buf += String(inst->cm.motor_l);
    inst->nh_.loginfo(buf.c_str());
    inst->cm.set_mot_c(ff2percent(msg.data));
    }),
  vib2_sub_("vibrator2", [](const std_msgs::UInt8& msg) {/*nop: not supported*/}),
  vib3_sub_("vibrator3", [](const std_msgs::UInt8& msg) {inst->cm.set_mot_l(ff2percent(msg.data));}),
  vib4_sub_("vibrator4", [](const std_msgs::UInt8& msg) {inst->cm.set_mot_r(ff2percent(msg.data));})
{
  inst = this;
  nh.subscribe(vib1_sub_);
  nh.subscribe(vib2_sub_);
  nh.subscribe(vib3_sub_);
  nh.subscribe(vib4_sub_);
}

void VibratorController_ace::init(){
  nh_.loginfo("initializing vibrator controller");
  cm.set_mot(0,0,0);
}

void VibratorController_ace::update() {
}
