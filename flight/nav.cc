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

#include "flight/global_defs.h"
#include "flight/hardware_defs.h"
#include "flight/config.h"
#include "flight/msg.h"
#include "flight/nav.h"
#include "navigation/navigation.h"
#include "airdata/airdata.h"

namespace {
/* Minimum number of satellites */
static constexpr int MIN_SAT_ = 12;
/* Navigation filter */
bfs::Ekf15State nav_filter_;
bool ekf_initialized = false;
Eigen::Vector3f imu_accel_, imu_gyro_, imu_mag_;
Eigen::Vector3f ekf_accel_, ekf_gyro_;
Eigen::Vector3f gnss_ned_vel_, ekf_ned_vel_;
Eigen::Vector3d gnss_lla_;
}  // namespace

void NavInit() {}

void NavRun(const SensorData &ref, NavData * const ptr) {
  if (!ptr) {return;}
  if (!ekf_initialized) {
    if (ref.imu.new_data && ref.gnss.new_data &&
        ref.gnss.num_sats > MIN_SAT_) {
      imu_accel_(0) = ref.imu.dlpf.accel_x_mps2;
      imu_accel_(1) = ref.imu.dlpf.accel_y_mps2;
      imu_accel_(2) = ref.imu.dlpf.accel_z_mps2;
      imu_gyro_(0) = ref.imu.dlpf.gyro_x_radps;
      imu_gyro_(1) = ref.imu.dlpf.gyro_y_radps;
      imu_gyro_(2) = ref.imu.dlpf.gyro_z_radps;
      imu_mag_(0) = ref.imu.dlpf.mag_x_ut;
      imu_mag_(1) = ref.imu.dlpf.mag_y_ut;
      imu_mag_(2) = ref.imu.dlpf.mag_z_ut;
      gnss_ned_vel_(0) = ref.gnss.north_vel_mps;
      gnss_ned_vel_(1) = ref.gnss.east_vel_mps;
      gnss_ned_vel_(2) = ref.gnss.down_vel_mps;
      gnss_lla_(0) = ref.gnss.lat_rad;
      gnss_lla_(1) = ref.gnss.lon_rad;
      gnss_lla_(2) = ref.gnss.alt_wgs84_m;
      nav_filter_.Initialize(imu_accel_, imu_gyro_, imu_mag_,
                             gnss_ned_vel_, gnss_lla_);
      ekf_initialized = true;
    }
  } else {
    if (ref.imu.new_data) {
      /* EKF time update */
      imu_accel_(0) = ref.imu.dlpf.accel_x_mps2;
      imu_accel_(1) = ref.imu.dlpf.accel_y_mps2;
      imu_accel_(2) = ref.imu.dlpf.accel_z_mps2;
      imu_gyro_(0) = ref.imu.dlpf.gyro_x_radps;
      imu_gyro_(1) = ref.imu.dlpf.gyro_y_radps;
      imu_gyro_(2) = ref.imu.dlpf.gyro_z_radps;
      nav_filter_.TimeUpdate(imu_accel_, imu_gyro_, FRAME_PERIOD_S);
    }
    if (ref.gnss.new_data) {
      /* EKF measurement update */
      gnss_ned_vel_(0) = ref.gnss.north_vel_mps;
      gnss_ned_vel_(1) = ref.gnss.east_vel_mps;
      gnss_ned_vel_(2) = ref.gnss.down_vel_mps;
      gnss_lla_(0) = ref.gnss.lat_rad;
      gnss_lla_(1) = ref.gnss.lon_rad;
      gnss_lla_(2) = ref.gnss.alt_wgs84_m;
      nav_filter_.MeasurementUpdate(gnss_ned_vel_, gnss_lla_);
    }
  }
  /* EKF data */
  ekf_accel_ = nav_filter_.accel_mps2();
  ekf_gyro_ = nav_filter_.gyro_radps();
  ekf_ned_vel_ = nav_filter_.ned_vel_mps();
  ptr->accel_x_mps2 = ekf_accel_(0);
  ptr->accel_y_mps2 = ekf_accel_(1);
  ptr->accel_z_mps2 = ekf_accel_(2);
  ptr->gyro_x_radps = ekf_gyro_(0);
  ptr->gyro_y_radps = ekf_gyro_(1);
  ptr->gyro_z_radps = ekf_gyro_(2);
  ptr->north_vel_mps = ekf_ned_vel_(0);
  ptr->east_vel_mps = ekf_ned_vel_(1);
  ptr->down_vel_mps = ekf_ned_vel_(2);
  ptr->lat_rad = nav_filter_.lat_rad();
  ptr->lon_rad = nav_filter_.lon_rad();
  ptr->alt_wgs84_m = nav_filter_.alt_m();
  ptr->pitch_rad = nav_filter_.pitch_rad();
  ptr->roll_rad = nav_filter_.roll_rad();
  ptr->heading_rad = nav_filter_.yaw_rad();
  /* Air data */
  ptr->alt_pres_m =
    bfs::PressureAltitude_m(ref.airdata.static_pres.dlpf.pres_pa);
  ptr->ias_mps = bfs::Ias_mps(ref.airdata.diff_pres.dlpf.pres_pa);
  ptr->eas_mps = bfs::Eas_mps(ref.airdata.diff_pres.dlpf.pres_pa,
                              ref.airdata.static_pres.dlpf.pres_pa);
  /* Derived data */
  ptr->gnd_spd_mps = std::sqrt(ptr->north_vel_mps * ptr->north_vel_mps +
                               ptr->east_vel_mps * ptr->east_vel_mps);
  ptr->gnd_track_rad = std::atan2(ptr->east_vel_mps, ptr->north_vel_mps);
  ptr->flight_path_rad = std::atan2(-1.0f * ptr->down_vel_mps,
                                    ptr->gnd_spd_mps);
}
