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

#include "flight/bfs-ins.h"
#include "global_defs.h"
#include "flight/msg.h"
#include "navigation.h"
#include "filter.h"
#include "eigen.h"

namespace {
static constexpr float FRAME_PERIOD_S = static_cast<float>(FRAME_PERIOD_MS) /
                                        1000.0f;
InsConfig cfg_;
bool ins_initialized_ = false;
static constexpr int8_t MIN_SAT_ = 7;
ImuData *imu_;
MagData *mag_;
GnssData *gnss_;
bfs::Lpf2p<float> ax_, ay_, az_, gx_, gy_, gz_, hx_, hy_, hz_;
Eigen::Vector3f accel_mps2_, gyro_radps_, mag_ut_, ned_vel_, rel_pos_ned_;
Eigen::Vector3d llh_;
bfs::Ekf15State ekf_;
float BASELINE_LEN_M = 0.0f;
float cur_baseline_len_m_ = 0.0f;
uint8_t init_counter_ = 0;
}

void BfsInsInit(const InsConfig &ref) {
  cfg_ = ref;
  ekf_.gnss_pos_ne_std_m(0.1f);
  ekf_.gnss_pos_d_std_m(0.2f);
  BASELINE_LEN_M = cfg_.antenna_baseline_m.norm();
}

void BfsInsRun(SensorData &ref, InsData * const ptr) {
  if (!ptr) {return;}
  ptr->initialized = ins_initialized_;
  if (!ins_initialized_) {
    /* Setup IMU source */
    switch (cfg_.imu_source) {
      #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
          defined(__FMU_R_V2_BETA__)
      case INS_IMU_VECTOR_NAV: {
        if (ref.vector_nav_imu.installed) {
          imu_ = &ref.vector_nav_imu;
        } else {
          MsgError("INS IMU source set to VectorNav, which is not installed");
        }
        break;
      }
      #endif
      case INS_IMU_FMU: {
        if (ref.fmu_imu.installed) {
          imu_ = &ref.fmu_imu;
        } else {
          MsgError("INS IMU source set to FMU, which is not installed");
        }
        break;
      }
    }
    /* Setup mag source */
    switch (cfg_.mag_source) {
      case INS_MAG_FMU: {
        if (ref.fmu_mag.installed) {
          mag_ = &ref.fmu_mag;
        } else {
          MsgError("INS mag source set to FMU, which is not installed");
        }
        break;
      }
      case INS_MAG_EXT_MAG: {
        if (ref.ext_mag.installed) {
          mag_ = &ref.ext_mag;
        } else {
          MsgError("INS mag source set to ext mag1, which is not installed");
        }
        break;
      }
    }
    /* Setup GNSS source */
    switch (cfg_.gnss_source) {
      #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
        defined(__FMU_R_V2_BETA__)
      case INS_GNSS_VECTOR_NAV: {
        if (ref.vector_nav_gnss.installed) {
          gnss_ = &ref.vector_nav_gnss;
        } else {
          MsgError("INS GNSS source set to VectorNav, which is not installed");
        }
        break;
      }
      #endif
      case INS_GNSS_EXT_GNSS1: {
        if (ref.ext_gnss1.installed) {
          gnss_ = &ref.ext_gnss1;
        } else {
          MsgError("INS GNSS source set to ext GNSS1, which is not installed");
        }
        break;
      }
      #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
          defined(__FMU_R_MINI_V1__)
      case INS_GNSS_EXT_GNSS2: {
        if (ref.ext_gnss2.installed) {
          gnss_ = &ref.ext_gnss2;
        } else {
          MsgError("INS GNSS source set to ext GNSS2, which is not installed");
        }
        break;
      }
      #endif
    }
    /* Init EKF */
    // Check for GNSS fix type if moving baseline is used. Skip this if using normal GPS
    if ((BASELINE_LEN_M != 0) && (gnss_->fix < 5)){
      return;
    }
    if ((imu_->new_data) && (mag_->new_data) && (gnss_->new_data) &&
        (gnss_->num_sats > MIN_SAT_)) {
      // Wait for initial conditions to pass several time before initializing the filter. Just to make sure everything is stabile
      if (init_counter_ < 10){
        init_counter_ ++;
        return;
      }
      accel_mps2_[0] = imu_->accel_mps2[0];
      accel_mps2_[1] = imu_->accel_mps2[1];
      accel_mps2_[2] = imu_->accel_mps2[2];
      gyro_radps_[0] = imu_->gyro_radps[0];
      gyro_radps_[1] = imu_->gyro_radps[1];
      gyro_radps_[2] = imu_->gyro_radps[2];
      mag_ut_[0] = mag_->mag_ut[0];
      mag_ut_[1] = mag_->mag_ut[1];
      mag_ut_[2] = mag_->mag_ut[2];
      ned_vel_[0] = gnss_->ned_vel_mps[0];
      ned_vel_[1] = gnss_->ned_vel_mps[1];
      ned_vel_[2] = gnss_->ned_vel_mps[2];
      llh_[0] = gnss_->lat_rad;
      llh_[1] = gnss_->lon_rad;
      llh_[2] = gnss_->alt_wgs84_m;
      rel_pos_ned_ [0] = float(gnss_->rel_pos_ned_m[0]);
      rel_pos_ned_ [1] = float(gnss_->rel_pos_ned_m[1]);
      rel_pos_ned_ [2] = float(gnss_->rel_pos_ned_m[2]);
      ekf_.Initialize(accel_mps2_, gyro_radps_, mag_ut_,
                      ned_vel_, llh_, rel_pos_ned_, cfg_.hardcoded_heading);
      /* Init DLPF */
      gx_.Init(cfg_.gyro_cutoff_hz, FRAME_RATE_HZ, ekf_.gyro_radps()[0]);
      gy_.Init(cfg_.gyro_cutoff_hz, FRAME_RATE_HZ, ekf_.gyro_radps()[1]);
      gz_.Init(cfg_.gyro_cutoff_hz, FRAME_RATE_HZ, ekf_.gyro_radps()[2]);
      ax_.Init(cfg_.accel_cutoff_hz, FRAME_RATE_HZ, ekf_.accel_mps2()[0]);
      ay_.Init(cfg_.accel_cutoff_hz, FRAME_RATE_HZ, ekf_.accel_mps2()[1]);
      az_.Init(cfg_.accel_cutoff_hz, FRAME_RATE_HZ, ekf_.accel_mps2()[2]);
      hx_.Init(cfg_.mag_cutoff_hz, MAG_RATE_HZ, mag_->mag_ut[0]);
      hy_.Init(cfg_.mag_cutoff_hz, MAG_RATE_HZ, mag_->mag_ut[1]);
      hz_.Init(cfg_.mag_cutoff_hz, MAG_RATE_HZ, mag_->mag_ut[2]);
      ins_initialized_ = true;
    }
  } else {
    if (imu_->new_data) {
      accel_mps2_[0] = imu_->accel_mps2[0];
      accel_mps2_[1] = imu_->accel_mps2[1];
      accel_mps2_[2] = imu_->accel_mps2[2];
      gyro_radps_[0] = imu_->gyro_radps[0];
      gyro_radps_[1] = imu_->gyro_radps[1];
      gyro_radps_[2] = imu_->gyro_radps[2];
      ekf_.TimeUpdate(accel_mps2_, gyro_radps_, FRAME_PERIOD_S);
    }
    if (gnss_->new_data) {
      ned_vel_[0] = gnss_->ned_vel_mps[0];
      ned_vel_[1] = gnss_->ned_vel_mps[1];
      ned_vel_[2] = gnss_->ned_vel_mps[2];
      llh_[0] = gnss_->lat_rad;
      llh_[1] = gnss_->lon_rad;
      llh_[2] = gnss_->alt_wgs84_m;
      rel_pos_ned_ [0] = float(gnss_->rel_pos_ned_m[0]);
      rel_pos_ned_ [1] = float(gnss_->rel_pos_ned_m[1]);
      rel_pos_ned_ [2] = float(gnss_->rel_pos_ned_m[2]);
      cur_baseline_len_m_ = rel_pos_ned_.norm();
      ekf_.MeasurementUpdate_gnss(ned_vel_, llh_);
      if ((gnss_->fix >= 5) && (abs(cur_baseline_len_m_ - BASELINE_LEN_M) < 0.1f )){
        ekf_.MeasurementUpdate_moving_base(rel_pos_ned_);
      }
    }
    ptr->pitch_rad = ekf_.pitch_rad();
    ptr->roll_rad = ekf_.roll_rad();
    ptr->heading_rad = ekf_.yaw_rad();
    ptr->alt_wgs84_m = ekf_.alt_m();
    ptr->accel_mps2[0] = ax_.Filter(ekf_.accel_mps2()[0]);
    ptr->accel_mps2[1] = ay_.Filter(ekf_.accel_mps2()[1]);
    ptr->accel_mps2[2] = az_.Filter(ekf_.accel_mps2()[2]);
    ptr->gyro_radps[0] = gx_.Filter(ekf_.gyro_radps()[0]);
    ptr->gyro_radps[1] = gy_.Filter(ekf_.gyro_radps()[1]);
    ptr->gyro_radps[2] = gz_.Filter(ekf_.gyro_radps()[2]);
    ptr->ned_vel_mps[0] = ekf_.ned_vel_mps()[0];
    ptr->ned_vel_mps[1] = ekf_.ned_vel_mps()[1];
    ptr->ned_vel_mps[2] = ekf_.ned_vel_mps()[2];
    ptr->lat_rad = ekf_.lat_rad();
    ptr->lon_rad = ekf_.lon_rad();
    if (mag_->new_data) {
      ptr->mag_ut[0] = hx_.Filter(mag_->mag_ut[0]);
      ptr->mag_ut[1] = hy_.Filter(mag_->mag_ut[1]);
      ptr->mag_ut[2] = hz_.Filter(mag_->mag_ut[2]);
    }
  }
}
