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

#include "flight/bfs_ekf.h"
#include "flight/global_defs.h"
#include "flight/hardware_defs.h"
#include "flight/msg.h"
#include "navigation.h"  // NOLINT
#include "filter.h"  // NOLINT

namespace {
/* Config */
BfsNavConfig cfg;
/* Initialized flag */
bool nav_initialized = false;
/* Navigation filter */
Ekf15State ekf;
/* IMU and GNSS data */
ImuData imu;
GnssData gnss;
/* Low pass filters */
Iir<float> gx;
Iir<float> gy;
Iir<float> gz;
Iir<float> ax;
Iir<float> ay;
Iir<float> az;
Iir<float> hx;
Iir<float> hy;
Iir<float> hz;
/* Minimum number of satellites */
static constexpr int MIN_SAT = 9;
/* Frame Period, s */
static constexpr float FRAME_PERIOD_S = FRAME_PERIOD_MS / 1000.0f;
/* Interim data */
Vector3f accel_mps2, gyro_radps, mag_ut, ned_vel_mps;
Vector3d llh;
}  // namespace

void BfsNavInit(const BfsNavConfig &ref) {
  cfg = ref;
}

void BfsNavRun(const SensorData &ref, InertialData * const ptr) {
  if (!ptr) {return;}
  ptr->healthy = nav_initialized;
  if (!nav_initialized) {
    switch (cfg.imu_source) {
      case EKF_IMU_MPU9250: {
        if (!ref.mpu9250_imu.installed) {
          MsgError("MPU-9250 selected for EKF IMU source, but not installed");
        }
        imu = ref.mpu9250_imu;
        break;
      }
      case EKF_IMU_VECTORNAV: {
        if (!ref.vector_nav_imu.installed) {
          MsgError("VectorNav selected for EKF IMU source, but not installed");
        }
        imu = ref.vector_nav_imu;
        break;
      }
    }
    switch (cfg.gnss_source_prim) {
      case EKF_GNSS_UBLOX3: {
        if (!ref.ublox3_gnss.installed) {
          MsgError("uBlox3 selected for EKF GNSS source, but not installed");
        }
        gnss = ref.ublox3_gnss;
        break;
      }
      case EKF_GNSS_UBLOX4: {
        if (!ref.ublox4_gnss.installed) {
          MsgError("uBlox4 selected for EKF GNSS source, but not installed");
        }
        gnss = ref.ublox4_gnss;
        break;
      }
      case EKF_GNSS_VECTORNAV: {
        if (!ref.vector_nav_gnss.installed) {
          MsgError("VectorNav selected for EKF GNSS source, but not installed");
        }
        gnss = ref.vector_nav_gnss;
        break;
      }
    }
    if ((imu.new_imu_data) && (imu.new_mag_data) && (gnss.new_data) &&
        (gnss.num_sats > MIN_SAT)) {
      /* Init IMU filters */
      gx.Init(cfg.gyro_cutoff_hz, FRAME_RATE_HZ, imu.gyro_radps[0]);
      gy.Init(cfg.gyro_cutoff_hz, FRAME_RATE_HZ, imu.gyro_radps[1]);
      gz.Init(cfg.gyro_cutoff_hz, FRAME_RATE_HZ, imu.gyro_radps[2]);
      ax.Init(cfg.accel_cutoff_hz, FRAME_RATE_HZ, imu.accel_mps2[0]);
      ay.Init(cfg.accel_cutoff_hz, FRAME_RATE_HZ, imu.accel_mps2[1]);
      az.Init(cfg.accel_cutoff_hz, FRAME_RATE_HZ, imu.accel_mps2[2]);
      hx.Init(cfg.mag_cutoff_hz, MPU9250_MAG_FRAME_RATE_HZ, imu.mag_ut[0]);
      hy.Init(cfg.mag_cutoff_hz, MPU9250_MAG_FRAME_RATE_HZ, imu.mag_ut[1]);
      hz.Init(cfg.mag_cutoff_hz, MPU9250_MAG_FRAME_RATE_HZ, imu.mag_ut[2]);
      /* Init EKF */
      gyro_radps[0] = imu.gyro_radps[0];
      gyro_radps[1] = imu.gyro_radps[1];
      gyro_radps[2] = imu.gyro_radps[2];
      accel_mps2[0] = imu.accel_mps2[0];
      accel_mps2[1] = imu.accel_mps2[1];
      accel_mps2[2] = imu.accel_mps2[2];
      mag_ut[0] = imu.mag_ut[0];
      mag_ut[1] = imu.mag_ut[1];
      mag_ut[2] = imu.mag_ut[2];
      ned_vel_mps[0] = gnss.ned_vel_mps[0];
      ned_vel_mps[1] = gnss.ned_vel_mps[1];
      ned_vel_mps[2] = gnss.ned_vel_mps[2];
      llh[0] = gnss.lat_rad;
      llh[1] = gnss.lon_rad;
      llh[2] = gnss.alt_wgs84_m;
      ekf.Initialize(accel_mps2, gyro_radps, mag_ut, ned_vel_mps, llh);
      nav_initialized = true;
    }
  } else {
    switch (cfg.imu_source) {
      case EKF_IMU_MPU9250: {
        imu = ref.mpu9250_imu;
        break;
      }
      case EKF_IMU_VECTORNAV: {
        imu = ref.vector_nav_imu;
        break;
      }
    }
    switch (cfg.gnss_source_prim) {
      case EKF_GNSS_UBLOX3: {
        gnss = ref.ublox3_gnss;
        break;
      }
      case EKF_GNSS_UBLOX4: {
        gnss = ref.ublox4_gnss;
        break;
      }
      case EKF_GNSS_VECTORNAV: {
        gnss = ref.vector_nav_gnss;
        break;
      }
    }
    /* EKF time update */
    if (imu.new_imu_data) {
      gyro_radps[0] = imu.gyro_radps[0];
      gyro_radps[1] = imu.gyro_radps[1];
      gyro_radps[2] = imu.gyro_radps[2];
      accel_mps2[0] = imu.accel_mps2[0];
      accel_mps2[1] = imu.accel_mps2[1];
      accel_mps2[2] = imu.accel_mps2[2];
      ekf.TimeUpdate(accel_mps2, gyro_radps, FRAME_PERIOD_S);
    }
    /* EKF measurement update */
    if (gnss.new_data) {
      ned_vel_mps[0] = gnss.ned_vel_mps[0];
      ned_vel_mps[1] = gnss.ned_vel_mps[1];
      ned_vel_mps[2] = gnss.ned_vel_mps[2];
      llh[0] = gnss.lat_rad;
      llh[1] = gnss.lon_rad;
      llh[2] = gnss.alt_wgs84_m;
      ekf.MeasurementUpdate(ned_vel_mps, llh);
    }
    /* EKF data */
    ptr->pitch_rad = ekf.pitch_rad();
    ptr->roll_rad = ekf.roll_rad();
    ptr->heading_rad = ekf.yaw_rad();
    ptr->alt_wgs84_m = ekf.alt_m();
    accel_mps2 = ekf.accel_mps2();
    gyro_radps = ekf.gyro_radps();
    ned_vel_mps = ekf.ned_vel_mps();
    ptr->lat_rad = ekf.lat_rad();
    ptr->lon_rad = ekf.lon_rad();
    ptr->ned_vel_mps[0] = ned_vel_mps[0];
    ptr->ned_vel_mps[1] = ned_vel_mps[1];
    ptr->ned_vel_mps[2] = ned_vel_mps[2];
    ptr->accel_mps2[0] = ax.Filter(accel_mps2[0]);
    ptr->accel_mps2[1] = ay.Filter(accel_mps2[1]);
    ptr->accel_mps2[2] = az.Filter(accel_mps2[2]);
    ptr->gyro_radps[0] = gx.Filter(gyro_radps[0]);
    ptr->gyro_radps[1] = gy.Filter(gyro_radps[1]);
    ptr->gyro_radps[2] = gz.Filter(gyro_radps[2]);
    if (imu.new_mag_data) {
      ptr->mag_ut[0] = hx.Filter(imu.mag_ut[0]);
      ptr->mag_ut[1] = hy.Filter(imu.mag_ut[1]);
      ptr->mag_ut[2] = hz.Filter(imu.mag_ut[2]);
    }
  }
}

