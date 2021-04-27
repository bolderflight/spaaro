/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
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

#include "flight/config.h"
#include "flight/hardware_defs.h"
#include "ams5915/ams5915.h"

// #else
// int8_t STATIC_PRES_CS = 26;
// #endif
/* Aircraft configuration */
AircraftConfig config = {
  .imu = {
    .odr = bfs::ImuConfig::ODR_50HZ,
    .accel_bias_mps2 = Eigen::Vector3f::Zero(),
    .mag_bias_ut = Eigen::Vector3f::Zero(),
    .accel_scale = Eigen::Matrix3f::Identity(),
    .mag_scale = Eigen::Matrix3f::Identity(),
    .rotation = Eigen::Matrix3f::Identity() * IMU_ROTATION
  },
  .gnss = {
    .baud = 921600,
    .sampling_period_ms = 20
  },
  .static_pres = {
    .sampling_period_ms = FRAME_PERIOD_MS
  },
  .diff_pres = {
    .sampling_period_ms = FRAME_PERIOD_MS
  }
};
