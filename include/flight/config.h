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

#ifndef INCLUDE_FLIGHT_CONFIG_H_
#define INCLUDE_FLIGHT_CONFIG_H_

#include "Eigen/Core"
#include "Eigen/Dense"
#include "ams5915/ams5915.h"
#include "effector/effector.h"

/* FMU Config */
static Eigen::Matrix3f FMU_ROTATION = Eigen::Matrix3f::Identity();
/* IMU Config */
static Eigen::Vector3f ACCEL_BIAS_MPS2 = Eigen::Vector3f::Zero();
static Eigen::Matrix3f ACCEL_SCALE_FACTOR = Eigen::Matrix3f::Identity();
static Eigen::Vector3f MAG_BIAS_UT = Eigen::Vector3f::Zero();
static Eigen::Matrix3f MAG_SCALE_FACTOR = Eigen::Matrix3f::Identity();
static constexpr std::array<float, 1> ACCEL_FILT_B = {0.039205f};
static constexpr std::array<float, 2> ACCEL_FILT_A = {1.0f, -0.96079f};
static constexpr std::array<float, 1> GYRO_FILT_B = {0.039205f};
static constexpr std::array<float, 2> GYRO_FILT_A = {1.0f, -0.96079f};
static constexpr std::array<float, 1> MAG_FILT_B = {0.039205f};
static constexpr std::array<float, 2> MAG_FILT_A = {1.0f, -0.96079f};
/* GNSS Receiver */
static constexpr HardwareSerial &GNSS_UART = Serial3;
static constexpr unsigned int GNSS_BAUD = 921600;
/* Pressure Transducers */
// #define HAVE_PITOT_STATIC
#ifdef HAVE_PITOT_STATIC
static constexpr bfs::Ams5915::Transducer DIFF_PRES_TRANSDUCER =
  bfs::Ams5915::AMS5915_0010_D;
#endif
static constexpr std::array<float, 1> DIFF_PRES_FILT_B = {0.039205f};
static constexpr std::array<float, 2> DIFF_PRES_FILT_A = {1.0f, -0.96079f};
static constexpr std::array<float, 1> STATIC_PRES_FILT_B = {0.039205f};
static constexpr std::array<float, 2> STATIC_PRES_FILT_A = {1.0f, -0.96079f};
/* Effectors */
static constexpr int NUM_SBUS_EFF = 0;
static constexpr int NUM_PWM_EFF = 0;
static std::array<bfs::Effector<10>, NUM_SBUS_EFF> sbus_effectors;
static std::array<bfs::Effector<10>, NUM_PWM_EFF> pwm_effectors;

#endif  // INCLUDE_FLIGHT_CONFIG_H_
