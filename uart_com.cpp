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
#include "uart_com.h"
#include "Arduino.h"
#include "stdlib.h"

#define UART Serial2

bool IsDecString(char * str) {
  char * endptr;
  strtol(str, &endptr, 10);

  if (*endptr == 0) {
    return true;
  }
  else {
    return false;
  }
}

unsigned short DecStringToDec(char * str){
  char * endptr;
  unsigned short val =strtol(str, &endptr, 10);

  if (*endptr == 0) {
    return val;
  }
  else
  {
    return false;
  }
}

uart_com::uart_com(ros::NodeHandle &nh):
    SensorReader(nh),
    error_pub_("error_count", &error_msg_) {
  nh.advertise(error_pub_);
}

void uart_com::begin(int baud_rate){
  UART.begin(baud_rate);
}

bool uart_com::parse_mot(){
  if(4 == words_len
  && IsDecString(words[1])
  && IsDecString(words[2])
  && IsDecString(words[3])){
    this->motor_r = DecStringToDec(words[1]);
    this->motor_c = DecStringToDec(words[2]);
    this->motor_l = DecStringToDec(words[3]);
    return true;
  }else{
    return false;
  }
}

bool uart_com::parse_mot_r(){
  if(2 == words_len
  && IsDecString(words[1])) {
    this->motor_r = DecStringToDec(words[1]);
    return true;
  }else{
    return false;
  }
}

bool uart_com::parse_mot_c(){
  if(2 == words_len
  && IsDecString(words[1])) {
    this->motor_c = DecStringToDec(words[1]);
    return true;
  }else{
    return false;
  }
}

bool uart_com::parse_mot_l(){
  if(2 == words_len
  && IsDecString(words[1])) {
    this->motor_l = DecStringToDec(words[1]);
    return true;
  }else{
    return false;
  }
}

bool uart_com::parse_thresh(){
  if(2 == words_len
  && IsDecString(words[1])){
    this->cap_thresh = DecStringToDec(words[1]);
    return true;
  }else{
    return false;
  }
}
bool uart_com::parse_sensi(){
  if(2 == words_len
  && IsDecString(words[1])){
    this->sensi = DecStringToDec(words[1]);
    return true;
  }else{
    return false;
  }
}

bool uart_com::parse_dat(){
  if(words_len != 12)return false;
  if(!IsDecString(words[1]))return false;
  if(!IsDecString(words[2]))return false;
  if(!IsDecString(words[3]))return false;
  if(!IsDecString(words[4]))return false;
  if(!IsDecString(words[5]))return false;
  if(!IsDecString(words[6]))return false;
  if(!IsDecString(words[7]))return false;
  if(!IsDecString(words[8]))return false;
  if(!IsDecString(words[9]))return false;
  if(!IsDecString(words[10]))return false;
  if(!IsDecString(words[11]))return false;

  this->touch =DecStringToDec(words[1]);
  this->capacitance =DecStringToDec(words[2]);
  this->cap_thresh =DecStringToDec(words[3]);
  this->motor_r =DecStringToDec(words[4]);
  this->motor_c =DecStringToDec(words[5]);
  this->motor_l =DecStringToDec(words[6]);
  this->switch_up =DecStringToDec(words[7]);
  this->switch_down =DecStringToDec(words[8]);
  this->switch_left =DecStringToDec(words[9]);
  this->switch_right =DecStringToDec(words[10]);
  this->switch_center =DecStringToDec(words[11]);

  return true;
}

bool uart_com::parse_dat_short(){
  if(words_len != 8)return false;
  if(!IsDecString(words[1]))return false;
  if(!IsDecString(words[2]))return false;
  if(!IsDecString(words[3]))return false;
  if(!IsDecString(words[4]))return false;
  if(!IsDecString(words[5]))return false;
  if(!IsDecString(words[6]))return false;
  if(!IsDecString(words[7]))return false;

  this->touch =DecStringToDec(words[1]);
  this->capacitance =DecStringToDec(words[2]);
  this->switch_up =DecStringToDec(words[3]);
  this->switch_down =DecStringToDec(words[4]);
  this->switch_left =DecStringToDec(words[5]);
  this->switch_right =DecStringToDec(words[6]);
  this->switch_center =DecStringToDec(words[7]);

  return true;
}

bool uart_com::parse_dat_shortest(){
  if(words_len != 3)return false;
  if(!IsDecString(words[1]))return false;
  if(!IsDecString(words[2]))return false;

  int code = DecStringToDec(words[1]);
  this->touch = (code >> 5) % 2;
  this->capacitance =DecStringToDec(words[2]);
  this->switch_up = (code >> 4) % 2;
  this->switch_down = (code >> 3) % 2;
  this->switch_left = (code >> 2) % 2;
  this->switch_right = (code >> 1) % 2;
  this->switch_center = (code >> 0) % 2;

  return true;
}

bool uart_com::parse_error(){
  if(words_len != 2)return false;
  if(!IsDecString(words[1]))return false;
  int code=DecStringToDec(words[1]);

  // ORE, NE, FE, PE
  this->error_count += ((code >> 3) % 2 * 1000000) + ((code >> 2) % 2 * 10000) + ((code >> 1) % 2 * 100) + code % 2;
}

void uart_com::StringCmdParse(char c){
  CMD_PARSE_FLAG =true;

  if (c == '\r'){
    //ignore '\r'
    CMD_PARSE_FLAG =false;
    return;
  }

  if(c != '\n'){
    CmdBuf[CmdBuf_wp++] = c;
  }else{
    CmdBuf[CmdBuf_wp] = '\0';
  }

  words_len=0;

  // If it exceeds the maximum, I'll go back first
  if (CmdBuf_wp >= CMD_BUF_MAX-1){
    CmdBuf_wp=0;
    CMD_PARSE_FLAG =false;
    return;
  }

  // Start command parsing when line feed code is received
  if(c == '\n'){
    char CmdCpy[CMD_BUF_MAX];
    strcpy(CmdCpy, CmdBuf);

    //Split string by ','
    char *str = strtok(CmdCpy, delim);
    while (str) {
      words[words_len] = str;
      words_len++;
      str = strtok(NULL, delim);
    }

    // Exit if there is no received character.
    if(words_len ==0)words[0] ="";

    if(strcmp(words[0] ,"DAT") ==0){
      if(this->parse_dat()){
        _last_update_millis = millis();
      }
    }else if(strcmp(words[0] ,"D") ==0){
      if(this->parse_dat_short()){
        _last_update_millis = millis();
      }
    }else if(strcmp(words[0] ,"d") ==0){
      if(this->parse_dat_shortest()){
        _last_update_millis = millis();
      }
    }else if(strcmp(words[0] ,"start") ==0){
      this->_started = true;
    }else if(strcmp(words[0] ,"stop") ==0){
      this->_started = false;
    }else if(strcmp(words[0] ,"MOT") ==0){
      this->parse_mot();
    }else if(strcmp(words[0] ,"R") ==0){
      this->parse_mot_r();
    }else if(strcmp(words[0] ,"C") ==0){
      this->parse_mot_c();
    }else if(strcmp(words[0] ,"L") ==0){
      this->parse_mot_l();
    }else if(strcmp(words[0] ,"THRESH") ==0){
      this->parse_thresh();
    }else if(strcmp(words[0] ,"SENSI") ==0){
      this->parse_sensi();
    }else if(strcmp(words[0] ,"E")==0){
      this->parse_error();
    }else if(strcmp(words[0] ,"") ==0){
    }else{
      //other_func();nop just handle error if any.
    }

    CmdBuf_wp=0;
    memset(CmdBuf,0,sizeof(CmdBuf));
  }
  CMD_PARSE_FLAG=false;
}

void uart_com::init(){
}

void uart_com::update(){
  while (UART.available()){
    char c = UART.read();
    this->StringCmdParse(c);
  }
}

void uart_com::start(){
  UART.println("start");
}
void uart_com::stop(){
  UART.println("stop");
}

bool uart_com::set_mot(int right, int center, int left){
  if(100 >= right && 100 >= center && 100 >= left){
    String buf="MOT,";
    buf += String(right) + ",";
    buf += String(center) + ",";
    buf += String(left);
    UART.println(buf);
    return true;
  }else{
    return false;
  }
}

bool uart_com::set_mot_r(int val){
  if(100 >= val) {
    //if (this->motor_r == val) return true;
    String buf="R,";
    buf += String(val);
    UART.println(buf);
    return true;
  }else{
    return false;
  }
}

bool uart_com::set_mot_c(int val){
  if(100 >= val) {
    //if (this->motor_c == val) return true;
    String buf="C,";
    buf += String(val);
    UART.println(buf);
    return true;
  }else{
    return false;
  }
}

bool uart_com::set_mot_l(int val){
  if(100 >= val) {
    //if (this->motor_l == val) return true;
    String buf="L,";
    buf += String(val);
    UART.println(buf);
    return true;
  }else{
    return false;
  }
}

bool uart_com::set_thresh(int thresh){
  if(127 >= thresh){
    String buf="THRESH,";
    buf += String(thresh);
    UART.println(buf);
    return true;
  }else{
    return false;
  }

}
bool uart_com::set_sensi(int sensi){
  switch(sensi){
    case 1:break;
    case 2:break;
    case 4:break;
    case 8:break;
    case 16:break;
    case 32:break;
    case 64:break;
    case 128:break;
    default:return false;
  }
  String buf="SENSI,";
  buf += String(sensi);
  UART.println(buf);
  return true;
}

bool uart_com::is_started(){
  return this->_started;
}

bool uart_com::is_alive(){
  unsigned long cur = millis();
  return ((cur - this->_last_update_millis) < 1000);
}

void uart_com::publish() {
  error_msg_.data = error_count;
  error_pub_.publish(&error_msg_);
}
