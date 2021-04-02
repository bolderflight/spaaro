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

#include "Eigen/Core"
#include "Eigen/Dense"
#include "flight/hardware_defs.h"
#include "flight/global_defs.h"
#include "flight/config.h"
#include "flight/msg.h"
#include "flight/imu.h"
#include "mpu9250/mpu9250.h"
#include "statistics/statistics.h"
#include "filter/filter.h"

namespace {
/* MPU-9250 fixed settings */
bfs::Mpu9250 imu_(&IMU_SPI_BUS, IMU_CS);
static constexpr bfs::Mpu9250::AccelRange ACCEL_RANGE_ =
  bfs::Mpu9250::ACCEL_RANGE_16G;
static constexpr bfs::Mpu9250::GyroRange GYRO_RANGE_ =
  bfs::Mpu9250::GYRO_RANGE_2000DPS;
static constexpr bfs::Mpu9250::DlpfBandwidth DLPF_ =
  bfs::Mpu9250::DLPF_BANDWIDTH_20HZ;
static constexpr unsigned int SRD_ = 1000 / FRAME_RATE_HZ - 1;
static Eigen::Matrix3f IMU_ROTATION_ =
  (Eigen::Matrix3f() << 0, 1, 0, -1, 0, 0, 0, 0, 1).finished();
/* Gyro bias estimation */
elapsedMillis time_ms_;
static constexpr float INIT_TIME_S_ = 5.0f;
bfs::RunningStats<float> gx_, gy_, gz_;
Eigen::Vector3f gyro_bias_radps_ = Eigen::Vector3f::Zero();
/* Filtering */
bfs::DigitalFilter1D<float, ACCEL_FILT_B.size(), ACCEL_FILT_A.size()>
  ax_filt(ACCEL_FILT_B, ACCEL_FILT_A),
  ay_filt(ACCEL_FILT_B, ACCEL_FILT_A),
  az_filt(ACCEL_FILT_B, ACCEL_FILT_A);
bfs::DigitalFilter1D<float, GYRO_FILT_B.size(), GYRO_FILT_A.size()>
  gx_filt(GYRO_FILT_B, GYRO_FILT_A),
  gy_filt(GYRO_FILT_B, GYRO_FILT_A),
  gz_filt(GYRO_FILT_B, GYRO_FILT_A);
bfs::DigitalFilter1D<float, MAG_FILT_B.size(), MAG_FILT_A.size()>
  hx_filt(MAG_FILT_B, MAG_FILT_A),
  hy_filt(MAG_FILT_B, MAG_FILT_A),
  hz_filt(MAG_FILT_B, MAG_FILT_A);
/* Data */
Eigen::Vector3f gyro_radps_, accel_mps2_, mag_ut_;
}  // namespace

void ImuInit() {
  MsgInfo("Initializing imu...");
  if (!imu_.Begin()) {
    MsgError("Unable to initialize communication with IMU.");
  }
  imu_.ApplyRotation(FMU_ROTATION * IMU_ROTATION_);
  if (!imu_.ConfigAccelRange(ACCEL_RANGE_)) {
    MsgError("Unable to set imu accel full-scale range.");
  }
  if (!imu_.ConfigGyroRange(GYRO_RANGE_)) {
    MsgError("Unable to set imu gyro full-scale range.");
  }
  if (!imu_.ConfigDlpf(DLPF_)) {
    MsgError("Unable to set imu DLPF bandwidth.");
  }
  if (!imu_.ConfigSrd(SRD_)) {
    MsgError("Unable to set imu sample rate divider.");
  }
  if (!imu_.EnableDrdyInt()) {
    MsgError("Unable to enable data ready interrupt.");
  }
  MsgInfo("done.\n");
  MsgInfo("Removing gyro bias and warming up filters...");
  time_ms_ = 0;
  unsigned int t1, t2;
  while (time_ms_ < INIT_TIME_S_ * 1000.0f) {
    if (imu_.Read()) {
      /* Gyro stats to remove bias */
      gx_.Update(imu_.gyro_x_radps());
      gy_.Update(imu_.gyro_y_radps());
      gz_.Update(imu_.gyro_z_radps());
      /* Accel and mag scale factor and bias corrections */
      accel_mps2_ = ACCEL_SCALE_FACTOR * imu_.accel_mps2() + ACCEL_BIAS_MPS2;
      mag_ut_ = MAG_SCALE_FACTOR * imu_.mag_ut() + MAG_BIAS_UT;
      /* Warm up accel and mag filters */
      ax_filt.Filter(accel_mps2_(0));
      ay_filt.Filter(accel_mps2_(1));
      az_filt.Filter(accel_mps2_(2));
      hx_filt.Filter(mag_ut_(0));
      hy_filt.Filter(mag_ut_(1));
      hz_filt.Filter(mag_ut_(2));
    }
  }
  /* Assign gyro bias */
  gyro_bias_radps_(0) = -gx_.mean();
  gyro_bias_radps_(1) = -gy_.mean();
  gyro_bias_radps_(2) = -gz_.mean();
  MsgInfo("done.\n");
}
void ImuRegisterDrdyIsr(void (*function)()) {
  imu_.DrdyCallback(IMU_DRDY, function);
}
void ImuRead(ImuData *const ptr) {
  if (!ptr) {return;}
  ptr->new_data = imu_.Read();
  if (ptr->new_data) {
    /* Apply scale factor and bias corrections */
    accel_mps2_ = ACCEL_SCALE_FACTOR * imu_.accel_mps2() + ACCEL_BIAS_MPS2;
    gyro_radps_ = imu_.gyro_radps() + gyro_bias_radps_;
    mag_ut_ = MAG_SCALE_FACTOR * imu_.mag_ut() + MAG_BIAS_UT;
    /* Output data */
    ptr->accel_x_mps2 = accel_mps2_(0);
    ptr->accel_y_mps2 = accel_mps2_(1);
    ptr->accel_z_mps2 = accel_mps2_(2);
    ptr->gyro_x_radps = gyro_radps_(0);
    ptr->gyro_y_radps = gyro_radps_(1);
    ptr->gyro_z_radps = gyro_radps_(2);
    ptr->mag_x_ut = mag_ut_(0);
    ptr->mag_y_ut = mag_ut_(1);
    ptr->mag_z_ut = mag_ut_(2);
    /* Output filtered data */
    ptr->dlpf.accel_x_mps2 = ax_filt.Filter(accel_mps2_(0));
    ptr->dlpf.accel_y_mps2 = ay_filt.Filter(accel_mps2_(1));
    ptr->dlpf.accel_z_mps2 = az_filt.Filter(accel_mps2_(2));
    ptr->dlpf.gyro_x_radps = gx_filt.Filter(gyro_radps_(0));
    ptr->dlpf.gyro_y_radps = gy_filt.Filter(gyro_radps_(1));
    ptr->dlpf.gyro_z_radps = gz_filt.Filter(gyro_radps_(2));
    ptr->dlpf.mag_x_ut = hx_filt.Filter(mag_ut_(0));
    ptr->dlpf.mag_y_ut = hy_filt.Filter(mag_ut_(1));
    ptr->dlpf.mag_z_ut = hz_filt.Filter(mag_ut_(2));
  }
}
