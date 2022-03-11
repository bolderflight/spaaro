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

#ifndef FLIGHT_CODE_INCLUDE_FLIGHT_IMU_H_
#define FLIGHT_CODE_INCLUDE_FLIGHT_IMU_H_

#include "flight/sensors.h"
#include "eigen.h"
#include <Eigen/Dense>

struct ImuConfig {
  SensorIface* dev;
  bool primary = false;
  Eigen::Vector3f accel_bias_mps2 = Eigen::Vector3f::Zero();
  Eigen::Matrix3f accel_scale = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
};

struct ImuData {
  Eigen::Vector3f gyro_radps;
  Eigen::Vector3f accel_mps2;
};

class ImuIface {
 public:
  virtual bool Config(const ImuConfig &ref) = 0;
  virtual ImuData data() = 0;
};

#endif  // FLIGHT_CODE_INCLUDE_FLIGHT_IMU_H_
