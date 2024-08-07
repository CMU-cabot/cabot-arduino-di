/*******************************************************************************
 * Copyright (c) 2020, 2023  Carnegie Mellon University, IBM Corporation, and others
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
#include "SensorReader.h"
#ifndef UART_COM_H_
#define UART_COM_H_
#define CMD_BUF_MAX (128)
#define MAX_LEN  64

class uart_com : SensorReader
{
private:
  // for parsing
  char CmdBuf[CMD_BUF_MAX];  // command buffer
  int CmdBuf_wp = 0;  // Number of stored in command buffer
  char * words[MAX_LEN];
  int words_len = 0;
  const char delim[2] = ",";
  bool CMD_PARSE_FLAG = false;
  bool StartFlg = false;

  uint32_t _last_update_millis = 0;
  bool _started = false;
  bool parse_mot();
  bool parse_mot_r();
  bool parse_mot_c();
  bool parse_mot_l();
  bool parse_thresh();
  bool parse_sensi();
  bool parse_dat();
  bool parse_dat_short();
  bool parse_dat_shortest();
  bool parse_error();
  void StringCmdParse(char c);

public:
  bool touch;
  int capacitance;
  int cap_thresh;
  int sensi = 2;
  int motor_r;
  int motor_c;
  int motor_l;
  int expected_motor_r;
  int expected_motor_c;
  int expected_motor_l;
  int resync_r;
  int resync_c;
  int resync_l;
  int switch_up;
  int switch_down;
  int switch_left;
  int switch_right;
  int switch_center;
  int servo_position;
  int error_count;
  explicit uart_com(cabot::Handle & ch);
  void init();
  void update();
  void begin(int baud_rate = 38400);
  void start();
  void stop();
  bool is_started();
  bool is_alive();
  bool set_mot(int right, int center, int left);
  bool set_mot_r(int val);
  bool set_mot_c(int val);
  bool set_mot_l(int val);
  bool set_thresh(int thresh);
  bool set_sensi(int sensi);
  bool set_servo_pos(int pos);
  bool set_servo_free(bool is_free);
  void publish();
  void check_feedback();
};

#endif  // UART_COM_H_
