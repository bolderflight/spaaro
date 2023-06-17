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

#include "global_defs.h"
#include "hardware_defs.h"
#include "drivers/fmu.h"
#include "eigen.h"
#include "statistics.h"
#include "flight/msg.h"
#if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
    defined(__FMU_R_V1__)
#include "mpu9250.h"
#include "bme280.h"
#elif defined(__FMU_R_MINI_V1__)
#include "mpu6500.h"
#include "lis3mdl.h"
#include "bmp3.h"
#endif

namespace {
/* Sensor objects */
#if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
    defined(__FMU_R_V1__)
bfs::Mpu9250 imu_(&SPI_BUS, IMU_CS);
bfs::Bme280 pres_(&SPI_BUS, PRES_CS);
#elif defined(__FMU_R_MINI_V1__)
bfs::Mpu6500 imu_(&SPI_BUS, IMU_CS);
bfs::Lis3mdl mag_(&SPI_BUS, MAG_CS);
bfs::Bmp3 pres_(&SPI_BUS, PRES_CS);
#endif
bool imu_installed_ = false, pres_installed_ = false, mag_installed_ = false;
uint8_t srd_;
/* Measurements */
Eigen::Vector3f accel_mps2_, gyro_radps_, mag_ut_;
/* Biases */
Eigen::Vector3f accel_bias_mps2_, gyro_bias_radps_, mag_bias_ut_;
/* Scale factor */
Eigen::Matrix3f accel_scale_, mag_scale_;
/* Rotation into vehicle frame */
Eigen::Matrix3f rotation_;
/* Rotation of sensors into board frame */
#if defined (__FMU_R_MINI_V1__)
Eigen::Matrix3f imu_board_rotation_{{0, -1, 0}, {-1, 0, 0}, {0, 0, -1}};
Eigen::Matrix3f mag_board_rotation_{{-1, 0, 0}, {0, -1, 0}, {0, 0, -1}};
#elif defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__)
Eigen::Matrix3f imu_board_rotation_ = Eigen::Matrix3f::Identity();
#elif defined(__FMU_R_V1__)
Eigen::Matrix3f imu_board_rotation_{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}};
#endif
/* Stats to estimate gyro biases */
bfs::RunningStats<float> gx_, gy_, gz_;
/* Timer to determine health */
elapsedMillis t_healthy_imu_, t_healthy_mag_, t_healthy_pres_;
}  // namespace

void FmuInit(const FmuConfig &cfg) {
  /* Initialize IMU */
  if (!imu_.Begin()) {
    MsgError("\tUnable to establish communication with FMU IMU");
  }
  if (!imu_.ConfigAccelRange(imu_.ACCEL_RANGE_4G)) {
    MsgError("\tUnable to configure FMU IMU accelerometer range");
  }
  if (!imu_.ConfigGyroRange(imu_.GYRO_RANGE_500DPS)) {
    MsgError("\tUnable to configure FMU IMU gyro range");
  }
  srd_ = 1000 / static_cast<uint8_t>(FRAME_RATE_HZ) - 1;
  if (!imu_.ConfigSrd(srd_)) {
    MsgError("\tUnable to configure FMU IMU sample rate divider");
  }
  switch (cfg.dlpf_hz) {
    #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
        defined(__FMU_R_MINI_V1__)
    case DLPF_BANDWIDTH_92HZ: {
      if (!imu_.ConfigDlpfBandwidth(imu_.DLPF_BANDWIDTH_92HZ)) {
        MsgError("\tUnable to configure FMU IMU DLPF");
      }
      break;
    }
    case DLPF_BANDWIDTH_41HZ: {
      if (!imu_.ConfigDlpfBandwidth(imu_.DLPF_BANDWIDTH_41HZ)) {
        MsgError("\tUnable to configure FMU IMU DLPF");
      }
      break;
    }
    #endif
    case DLPF_BANDWIDTH_20HZ: {
      if (!imu_.ConfigDlpfBandwidth(imu_.DLPF_BANDWIDTH_20HZ)) {
        MsgError("\tUnable to configure FMU IMU DLPF");
      }
      break;
    }
    case DLPF_BANDWIDTH_10HZ: {
      if (!imu_.ConfigDlpfBandwidth(imu_.DLPF_BANDWIDTH_10HZ)) {
        MsgError("\tUnable to configure FMU IMU DLPF");
      }
      break;
    }
    case DLPF_BANDWIDTH_5HZ: {
      if (!imu_.ConfigDlpfBandwidth(imu_.DLPF_BANDWIDTH_5HZ)) {
        MsgError("\tUnable to configure FMU IMU DLPF");
      }
      break;
    }
  }
  if (!imu_.EnableDrdyInt()) {
    MsgError("\tUnable to enable FMU IMU data ready interrupt");
  }
  imu_installed_ = true;
  /* Initialize pressure sensor */
  if (!pres_.Begin()) {
    MsgError("\tUnable establish communication with FMU static pressure sensor");
  }
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
    defined(__FMU_R_V1__)
  if(!pres_.ConfigPresOversampling(bfs::Bme280::OVERSAMPLING_8X)) {
    MsgError("\tUnable to configure FMU static pressure sensor pressure oversampling");
  }
  if(!pres_.ConfigTempOversampling(bfs::Bme280::OVERSAMPLING_1X)) {
    MsgError("\tUnable to configure FMU static pressure sensor temperature oversampling");
  }
  if(!pres_.ConfigHumidityOversampling(bfs::Bme280::OVERSAMPLING_1X)) {
    MsgError("\tUnable to configure FMU static pressure sensor humidity oversampling");
  }
  if(!pres_.ConfigFilterCoef(bfs::Bme280::FILTER_COEF_2)) {
    MsgError("\tUnable to configure FMU static pressure sensor filter coefficient");
  }
  if(!pres_.ConfigStandbyTime(bfs::Bme280::STANDBY_TIME_0_5_MS)) {
    MsgError("\tUnable to configure FMU static pressure sensor standby time");
  }
  #elif defined(__FMU_R_MINI_V1__)
  if(!pres_.ConfigOsMode(bfs::Bmp3::OS_MODE_PRES_8X_TEMP_1X)) {
    MsgError("\tUnable to configure FMU static pressure sensor oversampling");
  }
  if(!pres_.ConfigFilterCoef(bfs::Bmp3::FILTER_COEF_2)) {
    MsgError("\tUnable to configure FMU static pressure sensor filter coefficient");
  }
  #endif
  pres_installed_ = true;
  /* Initialize magnetometer */
  #if defined(__FMU_R_MINI_V1__)
  if (!mag_.Begin()) {
    MsgError("\tUnable establish communication with FMU magnetometer");
  }
  if (!mag_.ConfigRange(bfs::Lis3mdl::RANGE_16GS)) {
    MsgError("\tUnable to configure FMU magnetometer range");
  }
  if (!mag_.ConfigOdr(bfs::Lis3mdl::ODR_155HZ)) {
    MsgError("\tUnable to configure FMU magnetometer data rate");
  }
  #endif
  mag_installed_ = true;
  /* Save config */
  for (int8_t m = 0; m < 3; m++) {
    accel_bias_mps2_[m] = cfg.accel_bias_mps[m];
    mag_bias_ut_[m] = cfg.mag_bias_ut[m];
    for (int8_t n = 0; n < 3; n++) {
      accel_scale_(m ,n) = cfg.accel_scale[m][n];
      mag_scale_(m ,n) = cfg.mag_scale[m][n];
      rotation_(m ,n) = cfg.rotation[m][n];
    }
  }
}

void FmuCal() {
 if (imu_.Read()) {
  gyro_radps_[0] = imu_.gyro_x_radps();
  gyro_radps_[1] = imu_.gyro_y_radps();
  gyro_radps_[2] = imu_.gyro_z_radps();
  gyro_radps_ = imu_board_rotation_ * rotation_ * gyro_radps_;
  gx_.Update(gyro_radps_[0]);
  gy_.Update(gyro_radps_[1]);
  gz_.Update(gyro_radps_[2]);
  gyro_bias_radps_[0] = -gx_.mean();
  gyro_bias_radps_[1] = -gy_.mean();
  gyro_bias_radps_[2] = -gz_.mean();
 }
}

void FmuRead(ImuData * const imu, MagData * const mag, PresData * const pres) {
  /* Installed sensors */
  /* We can just assign this, since it's required to have the FMU sensors installed...*/
  imu->installed = imu_installed_;
  mag->installed = mag_installed_;
  pres->installed = pres_installed_;
  /* Read IMU */
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_V1__)
  imu->new_data = imu_.Read();
  if (imu->new_data) {
    t_healthy_imu_ = 0;
    accel_mps2_[0] = imu_.accel_x_mps2();
    accel_mps2_[1] = imu_.accel_y_mps2();
    accel_mps2_[2] = imu_.accel_z_mps2();
    accel_mps2_ = imu_board_rotation_ * accel_scale_ * rotation_ * accel_mps2_ +
                  accel_bias_mps2_;
    imu->accel_mps2[0] = accel_mps2_[0];
    imu->accel_mps2[1] = accel_mps2_[1];
    imu->accel_mps2[2] = accel_mps2_[2];
    gyro_radps_[0] = imu_.gyro_x_radps();
    gyro_radps_[1] = imu_.gyro_y_radps();
    gyro_radps_[2] = imu_.gyro_z_radps();
    gyro_radps_ = imu_board_rotation_ * rotation_ * gyro_radps_ +
                  gyro_bias_radps_;
    imu->gyro_radps[0] = gyro_radps_[0];
    imu->gyro_radps[1] = gyro_radps_[1];
    imu->gyro_radps[2] = gyro_radps_[2];
    imu->die_temp_c = imu_.die_temp_c();
    mag->die_temp_c = imu->die_temp_c;
    mag->new_data = imu_.new_mag_data();
    if (mag->new_data ) {
      t_healthy_mag_ = 0;
      mag_ut_[0] = imu_.mag_x_ut();
      mag_ut_[1] = imu_.mag_y_ut();
      mag_ut_[2] = imu_.mag_z_ut();
      mag_ut_ = imu_board_rotation_ * mag_scale_ * rotation_ * mag_ut_ +
                mag_bias_ut_;
      mag->mag_ut[0] = mag_ut_[0];
      mag->mag_ut[1] = mag_ut_[1];
      mag->mag_ut[2] = mag_ut_[2];
    }
  }
  imu->healthy = (t_healthy_imu_ < 10 * FRAME_PERIOD_MS);
  mag->healthy = (t_healthy_imu_ < 10 * FMU_MAG_PERIOD_MS);
  #elif defined(__FMU_R_MINI_V1__)
  imu->new_data = imu_.Read();
  if (imu->new_data) {
    t_healthy_imu_ = 0;
    accel_mps2_[0] = imu_.accel_x_mps2();
    accel_mps2_[1] = imu_.accel_y_mps2();
    accel_mps2_[2] = imu_.accel_z_mps2();
    accel_mps2_ = imu_board_rotation_ * accel_scale_ * rotation_ * accel_mps2_ +
                  accel_bias_mps2_;
    imu->accel_mps2[0] = accel_mps2_[0];
    imu->accel_mps2[1] = accel_mps2_[1];
    imu->accel_mps2[2] = accel_mps2_[2];
    gyro_radps_[0] = imu_.gyro_x_radps();
    gyro_radps_[1] = imu_.gyro_y_radps();
    gyro_radps_[2] = imu_.gyro_z_radps();
    gyro_radps_ = imu_board_rotation_ * rotation_ * gyro_radps_ +
                  gyro_bias_radps_;
    imu->gyro_radps[0] = gyro_radps_[0];
    imu->gyro_radps[1] = gyro_radps_[1];
    imu->gyro_radps[2] = gyro_radps_[2];
    imu->die_temp_c = imu_.die_temp_c();
  }
  /* Read mag */
  mag->new_data = mag_.Read();
  if (mag->new_data) {
    t_healthy_mag_ = 0;
    mag_ut_[0] = mag_.mag_x_ut();
    mag_ut_[1] = mag_.mag_y_ut();
    mag_ut_[2] = mag_.mag_z_ut();
    mag_ut_ = mag_board_rotation_ * mag_scale_ * rotation_ * mag_ut_ +
              mag_bias_ut_;
    mag->mag_ut[0] = mag_ut_[0];
    mag->mag_ut[1] = mag_ut_[1];
    mag->mag_ut[2] = mag_ut_[2];
  }
  imu->healthy = (t_healthy_imu_ < 10 * FRAME_PERIOD_MS);
  mag->healthy = (t_healthy_imu_ < 10 * FRAME_PERIOD_MS);
  #endif
  /* Read pres */
  pres->new_data = pres_.Read();
  if (pres->new_data) {
    t_healthy_pres_ = 0;
    pres->pres_pa = pres_.pres_pa();
    pres->die_temp_c = pres_.die_temp_c();
  }
  pres->healthy = (t_healthy_imu_ < 10 * FRAME_PERIOD_MS);
}
