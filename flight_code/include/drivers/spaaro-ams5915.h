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

#ifndef FLIGHT_CODE_INCLUDE_DRIVERS_AMS5915_H_
#define FLIGHT_CODE_INCLUDE_DRIVERS_AMS5915_H_

#include "global_defs.h"
#include "hardware_defs.h"
#include "ams5915.h"
#include "statistics.h"

class SpaaroAms5915 {
 public:
  explicit SpaaroAms5915(TwoWire *i2c) : i2c_(i2c) {}
  void Init(const PresConfig &cfg);
  void Cal();
  void Read(PresData * const data);

 private:
  bool installed_ = false;
  bool is_static_pres_ = false;
  elapsedMillis t_healthy_ms_;
  PresTransducer transducer_;
  TwoWire  *i2c_;
  bfs::Ams5915 pres_;
  bfs::RunningStats<float> bias_;
};

#endif  // FLIGHT_CODE_INCLUDE_DRIVERS_AMS5915_H_
