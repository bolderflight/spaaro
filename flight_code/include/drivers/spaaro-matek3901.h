/*
* Tuan Luong    
* tdluong@crimson.ua.edu
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

#ifndef FLIGHT_CODE_INCLUDE_DRIVERS_MATEK3901_H_
#define FLIGHT_CODE_INCLUDE_DRIVERS_MATEK3901_H_

#include "global_defs.h"
#include "hardware_defs.h"
#include "matek3901.h"

class SpaaroMatek3901 {
 public:
  SpaaroMatek3901(HardwareSerial *bus) : opflow_(bus) {}
  void Init(const OpFlowConfig &cfg);
  void Read(OpFlowData * const data);

 private:
  bool installed_ = false;
  static constexpr int8_t UPDATE_PERIOD_MS_ = 20;
  elapsedMillis t_healthy_ms_;
  bfs::Matek3901 opflow_;
};

#endif  // FLIGHT_CODE_INCLUDE_DRIVERS_MATEK3901_H_
