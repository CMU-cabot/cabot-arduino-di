/*******************************************************************************
 * Copyright (c) 2020, 2022  Carnegie Mellon University
 * Copyright (c) 2024  ALPS ALPINE CO.,LTD.
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

#include "CaBotHandle.hpp"  // NOLINT

// #define DEBUG 1

namespace cabot
{
Handle::Handle() {mBaudRate = 0;}

Handle::~Handle() {}

void Handle::setBaudRate(uint32_t rate) {mBaudRate = rate;}

void Handle::init()
{
  mConnected = false;
  mConnecting = false;
  mTimeOffset = 0;
  mTime.sec = 0;
  mTime.nsec = 0;
  mTimeOffset = 0;
  mTimeMillis = 0;
  state = 0;
  header_count = 0;
  size = 0;
  size_count = 0;
  cmd = 0;
  count = 0;
  if (mBaudRate == 0) {
    return;
  }
  Serial.begin(mBaudRate);
  while (!Serial) {
  }
  mConnected = true;
}

bool Handle::connected() {return mConnected;}

void Handle::spinOnce()
{
  static uint8_t cmd = 0;
  uint8_t * data;
  int count = readCommand(&cmd, &data);
  if (count < 0) {return;}

  if (cmd == 0x01 && count == 8) {
    // time sync command
    static char buff[128];
    // get the half of the turn around time
    // outbound and inbound transmission time is expected to be equal
    // the PC will return its time when it recevied the sync command
    // so the expected Arduino time is the returned time + the half of the turn
    // around time
    uint32_t ms = millis();
    int32_t turn_around_time = (ms - mSyncTime);
    Time newTime;
    newTime.sec = parseUInt32(data);
    newTime.nsec = parseUInt32(data + 4);
    int32_t back_time = turn_around_time / 2;
    uint32_t newTimeOffset = mSyncTime + back_time;

    // calcurate the time jump by the new time base
    Time prev = _now(mTime, ms, mTimeOffset);
    Time current = _now(newTime, ms, newTimeOffset);
    int32_t jump = timeDiff(current, prev);
    int32_t temp = 0;

    bool need_to_update = false;
    if (!is_synchronized()) {
      // initial time sync
      need_to_update = true;
    } else if (abs(jump) < 100000L) {
      // this can reduce the clock jumping around the ground truth, but not sure
      // why temp should not be bigger than back_time, if it exeeds back_time
      // _now computation overflows (ms - newTimeOffset going to be minus)
      temp = jump / 2L;
      newTimeOffset += temp;
      current = _now(newTime, ms, newTimeOffset);
      jump = timeDiff(current, prev);
      need_to_update = true;
    } else {
      snprintf(
        buff, sizeof(buff), "large time jump %u.%09u -> %u.%09u",
        prev.sec, prev.nsec, current.sec, current.nsec);
      logwarn(buff);
    }

    // #ifdef DEBUG
    snprintf(
      buff, sizeof(buff),
      "sync,%u,%u.%03u,%u.%03u,%u.%03u,%d,%d,%d,%d", ms,
      newTime.sec, newTime.nsec / 1000000, prev.sec, prev.nsec / 1000000,
      current.sec, current.nsec / 1000000, jump, turn_around_time,
      back_time, temp);
    loginfo(buff);
    // #endif

    if (need_to_update) {
      mTime = newTime;
      mTimeOffset = newTimeOffset;
      mTimeMillis = ms;
    }
  }
  if (0x20 <= cmd && cmd <= 0x23 && count == 1) {
    // feedback to the host
    sendCommand(cmd, data, 1);
    // vibration commands
    for (int i = 0; i < CALLBACK_NUMS; i++) {
      if (callbacks[i].cmd == cmd) {
        callbacks[i].callback(data[0]);
      }
    }
  }
  if (cmd == 0x36 && count == 2) {
    int16_t value = decode_binary_16bit(data);
    for (int i = 0; i < CALLBACK_NUMS; i++) {
      if (callbacks[i].cmd == cmd) {
        callbacks[i].callback_i16(value);
      }
    }
  }
  if (cmd == 0x37 && count == 1) {
    for (int i = 0; i < CALLBACK_NUMS; i++) {
      if (callbacks[i].cmd == cmd) {
        callbacks[i].callback_b((bool)data[0]);
      }
    }
  }
  cmd = 0;  // reset cmd
}

uint16_t Handle::decode_binary_16bit(uint8_t *ptr) {
  uint16_t bytel = 0;
  uint16_t byteh = 0;
  bytel = *ptr & 0x00FF;
  byteh = (*++ptr << 8) & 0xFF00;
  return byteh | bytel;
}

void Handle::subscribe(uint8_t cmd, void (* callback)(const uint8_t))
{
  Callback temp;
  temp.cmd = cmd;
  temp.callback = callback;
  callbacks[callback_count++] = temp;
}

void Handle::subscribe(uint8_t cmd, void (* callback)(const uint16_t))
{
  Callback temp;
  temp.cmd = cmd;
  temp.callback_ui16 = callback;
  callbacks[callback_count++] = temp;
}

void Handle::subscribe(uint8_t cmd, void (* callback)(const int16_t))
{
  Callback temp;
  temp.cmd = cmd;
  temp.callback_i16 = callback;
  callbacks[callback_count++] = temp;
}

void Handle::subscribe(uint8_t cmd, void (* callback)(const bool))
{
  Callback temp;
  temp.cmd = cmd;
  temp.callback_b = callback;
  callbacks[callback_count++] = temp;
}

void Handle::logdebug(char * text) {sendCommand(0x02, text, strlen(text));}

void Handle::loginfo(const char * text) {sendCommand(0x03, text, strlen(text));}

void Handle::logwarn(const char * text) {sendCommand(0x04, text, strlen(text));}

bool Handle::getParam(const char * name, int * out, size_t num, int timeout_ms)
{
  sendCommand(0x08, name, strlen(name));
  uint8_t cmd = 0x08;
  uint8_t * ptr;
  int count = 0;
  int read_count = 0;
  uint32_t start = millis();
  while ((read_count = readCommand(&cmd, &ptr)) < 0) {
    count += 1;
    if (count % 100 == 0) {
      if (millis() - start > timeout_ms) {
        // loginfo("timeout");
        return false;
      }
    }
  }
  if (read_count == 0) {
    // cannot find parameter
    return false;
  }
  for (size_t i = 0; i < num; i++) {
    out[i] = parseUInt32(ptr + i * 4);
  }

  return true;
}

void Handle::publish(
  uint8_t cmd, char * data,
  size_t num)                     // prevent from no wrapping
{
  sendCommand(cmd, reinterpret_cast<uint8_t *>(data), num);
}

void Handle::publish(
  uint8_t cmd, uint8_t * data,
  size_t num)                     // prevent from no wrapping
{
  sendCommand(cmd, data, num);
}

void Handle::publish(uint8_t cmd, int8_t * data, size_t num)
{
  sendCommand(cmd, reinterpret_cast<uint8_t *>(data), num);
}

void Handle::publish(uint8_t cmd, float * data, size_t num)
{
  uint8_t temp[128];
  for (int i = 0; i < num; i++) {
    toBytes(data[i], temp + i * 4);
  }
  sendCommand(cmd, temp, num * 4);
}

void Handle::publish(uint8_t cmd, int8_t data)
{
  uint8_t buff[1];
  toBytes(data, buff, 1);
  sendCommand(cmd, buff, 1);
}

void Handle::publish(uint8_t cmd, uint8_t data)
{
  uint8_t buff[1];
  toBytes(data, buff, 1);
  sendCommand(cmd, buff, 1);
}

void Handle::publish(uint8_t cmd, int16_t data)
{
  uint8_t buff[2];
  toBytes(data, buff, 2);
  sendCommand(cmd, buff, 2);
}

void Handle::publish(uint8_t cmd, uint16_t data)
{
  uint8_t buff[2];
  toBytes(data, buff, 2);
  sendCommand(cmd, buff, 2);
}

void Handle::publish(uint8_t cmd, float data)
{
  uint8_t buff[4];
  toBytes(data, buff);
  sendCommand(cmd, buff, 4);
}

void Handle::sync()
{
  uint8_t buff[8];

  if (is_synchronized()) {
    Time current = now();
    toBytes(current.sec, buff, 4);
    toBytes(current.nsec, buff + 4, 4);
  } else {
    toBytes(0, buff, 4);
    toBytes(0, buff + 4, 4);
  }
  mSyncTime = millis();
  sendCommand(0x01, buff, 8);
}

bool Handle::is_synchronized() {return mTimeOffset > 0;}

Time Handle::now() {return _now(mTime, millis(), mTimeOffset);}

/* private */

Time Handle::_now(Time base, uint32_t ms, uint32_t offset)
{
  Time current;
  int32_t diff = (ms - offset);  // mDelayRate;

  current.sec = base.sec + (diff / 1000UL);
  current.nsec = base.nsec + (diff % 1000UL) * 1000000UL;
  if (current.nsec > 1000000000UL) {
    current.nsec -= 1000000000UL;
    current.sec += 1;
  }
  return current;
}

/*
  returns time diff in microseconds (a-b)
  assumes time a and b are close enough
 */
int32_t Handle::timeDiff(Time a, Time b)
{
  int32_t dsec = a.sec - b.sec;
  int32_t nsec = (a.nsec / 1000000L) - (b.nsec / 1000000L);

  int32_t diff = dsec * 1000L + nsec;
  return diff;
}

/*
  readCommand will read one byte from serial and change its reading state

  @param expect: a pointer to a command value to be expected
    if *expect == 0 then read any command and set the read cmd value to *expect
    if *expect != 0 then return data only if the read command equals to *expect

  @param ptr: ptr to byte array data

  return
    the size of the data for the command (more than or equals to 0) if the
  command is parsed correctly -1 if the command is not parsed or in the middle
  of command
*/
size_t Handle::readCommand(uint8_t * expect, uint8_t ** ptr)
{
  static int DATA_MAX_SIZE_BYTE = 1;
  static uint8_t buffer[16];
  static char buff[48];

  if (Serial.available() == 0) {
    return -1;
  }
  int received = Serial.read();
  if (received < 0) {return -1;}
#ifdef DEBUG
  // print out read command state
  if (state != 0 || received != 0) {
    static char buff[48];
    snprintf(
      buff, 48, "%02x %d %d %x %x %d %d %d", received, state,
      header_count, cmd, *expect, size, size_count, count);
    loginfo(buff);
  }
#endif
  if (state == 0) {
    // find two consequtive 0xAA bytes
    if (received == 0xAA) {
      header_count += 1;
      if (header_count == 2) {
        header_count = 0;
        state = 1;
      }
      cmd = 0;
    } else {
      header_count = 0;
    }
  } else if (state == 1) {
    // get the cmd value
    cmd = received;

    if (*expect == cmd || *expect == 0) {
      // set the cmd value to expect
      *expect = cmd;
      state = 2;
      size = 0;
      size_count = 0;
    } else {
      // if the cmd is different from the expected, then reset
      state = 0;
      return -1;
    }
  } else if (state == 2) {
    // read cmd data size
    size = (received & 0xFF) << size_count * 8;
    size_count += 1;
    if (size_count == DATA_MAX_SIZE_BYTE) {
      if (sizeof(buffer) < size) {
        state = 0;
        snprintf(buff, 48, "size is too big cmd=%d, size=%d", cmd, size);
        logwarn(buff);
        return 0;
      } else if (size == 0) {
        // if size is zero then skip to checksum
        state = 4;
      } else {
        state = 3;
      }
    }
    count = 0;
  } else if (state == 3) {
    // read data
    buffer[count] = received;
    count += 1;
    if (count == size) {
      state = 4;
    }
  } else if (state == 4) {
    state = 0;
    if (received == checksum(buffer, size)) {
      *ptr = buffer;
      return size;
    } else {
      // if checksum is not matched, assumes there is no data
      snprintf(buff, 48, "check sum is not matched");
      logwarn(buff);
      return 0;
    }
  }
  return -1;
}

bool cabot::Handle::sendCommand(uint8_t type, const uint8_t * data, size_t num)
{
  static uint8_t buffer[256 + 6];
  if (256 < num) {
    return false;
  }

  buffer[0] = 0xAA;
  buffer[1] = 0xAA;
  buffer[2] = type;
  buffer[3] = num & 0xFF;
  buffer[4] = (num >> 8) & 0xFF;
  for (size_t i = 0; i < num; i++) {
    buffer[i + 5] = data[i];
  }

  buffer[num + 5] = checksum(data, num);

  size_t written = 0;
  while (written < num + 6) {
    if (Serial.availableForWrite() > 0) {
      written += Serial.write(buffer + written, num + 6 - written);
    }
  }
  return true;
}

bool cabot::Handle::sendCommand(uint8_t type, const char * data, size_t num)
{
  return sendCommand(type, reinterpret_cast<const uint8_t *>(data), num);
}

uint8_t Handle::checksum(const uint8_t * data, size_t num)
{
  uint8_t temp = 0;
  for (size_t i = 0; i < num; i++) {
    temp += data[i];
  }
  return 0xFF - (0xFF & temp);
}

typedef struct
{
  uint8_t c1;
  uint8_t c2;
  uint8_t c3;
  uint8_t c4;
} convert_t;

// bit shift over 16bit does not work
uint32_t Handle::parseUInt32(uint8_t * ptr)
{
  convert_t temp = {ptr[0], ptr[1], ptr[2], ptr[3]};
  return *reinterpret_cast<uint32_t *>(&temp);
}

void Handle::toBytes(uint32_t v, uint8_t * ptr, size_t num)
{
  for (int i = 0; i < num; i++) {
    ptr[i] = (v >> 8 * i) & 0xFF;
  }
}

void Handle::toBytes(float v, uint8_t * ptr)
{
  convert_t * temp = reinterpret_cast<convert_t *>(&v);
  ptr[0] = temp->c1;
  ptr[1] = temp->c2;
  ptr[2] = temp->c3;
  ptr[3] = temp->c4;
}

}  // namespace cabot
