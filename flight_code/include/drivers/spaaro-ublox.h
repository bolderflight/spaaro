/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#ifndef FLIGHT_CODE_INCLUDE_DRIVERS_UBLOX_H_
#define FLIGHT_CODE_INCLUDE_DRIVERS_UBLOX_H_

#include "global_defs.h"
#include "hardware_defs.h"
#include "ubx.h"

class SpaaroUbx {
 public:
  SpaaroUbx(HardwareSerial* bus) : gnss_(bus) {}
  void Init(const GnssConfig &cfg);
  void Read(GnssData * const data);

 private:
  bool installed_ = false;
  static constexpr int16_t HEALTHY_PERIOD_MS_ = 1000;
  elapsedMillis t_healthy_ms_;
  bfs::Ubx gnss_;
};

#endif  // FLIGHT_CODE_INCLUDE_DRIVERS_UBLOX_H_
