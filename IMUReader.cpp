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

#include "IMUReader.h"

#define D2R 0.0174532925

static float angle_constrain(float angle){
  while (angle > 180) {
    angle -= 360;
  }
  while (angle < -180) {
    angle +=360;
  }
  return angle;
}

IMUReader::IMUReader(ros::NodeHandle &nh):
  SensorReader(nh),
  imu_pub_("imu", &imu_msg_)
{
  nh_.advertise(imu_pub_);
}

void IMUReader::init() {
  if(!imu_.begin())
  {
    nh_.loginfo("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  imu_.setExtCrystalUse(true);
  imu_msg_.orientation_covariance[0] = 0.1;
  imu_msg_.orientation_covariance[4] = 0.1;
  imu_msg_.orientation_covariance[8] = 0.1;
}

void IMUReader::update() {
  imu::Quaternion q = imu_.getQuat();

  imu_msg_.orientation.x = q.x();
  imu_msg_.orientation.y = q.y();
  imu_msg_.orientation.z = q.z();
  imu_msg_.orientation.w = q.w();

  imu::Vector<3> xyz = imu_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  imu_msg_.angular_velocity.x = xyz.x()*D2R;
  imu_msg_.angular_velocity.y = xyz.y()*D2R;
  imu_msg_.angular_velocity.z = xyz.z()*D2R;
    
  xyz = imu_.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  imu_msg_.linear_acceleration.x = xyz.x();
  imu_msg_.linear_acceleration.y = xyz.y();
  imu_msg_.linear_acceleration.z = xyz.z();

  // publish
  imu_msg_.header.stamp = nh_.now();
  imu_msg_.header.frame_id = "imu_frame";
  imu_pub_.publish( &imu_msg_ );
}
