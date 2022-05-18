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

#include "flight/config.h"
#include "flight/global_defs.h"

/* Debug */
bool DEBUG = true;
/* Aircraft config */
AircraftConfig config = {
  .sensor = {
    .drdy_source = DRDY_MPU9250,
    // .sbus = {
    //   .installed = true
    // },
    .mpu9250 = {
      .accel_range_g = Mpu9250::ACCEL_RANGE_16G,
      .gyro_range_dps = Mpu9250::GYRO_RANGE_2000DPS,
      .dlpf_hz = Mpu9250::DLPF_BANDWIDTH_20HZ,
      .accel_bias_mps2 = Vector3f::Zero(),
      .mag_bias_ut = Vector3f::Zero(),
      .accel_scale = Matrix3f::Identity(),
      .mag_scale = Matrix3f::Identity(),
      .rotation = Matrix3f::Identity()
    },
    // .vector_nav = {
    //   .device = VECTORNAV_VN300,
    //   .accel_filt_window = 4,
    //   .gyro_filt_window = 4,
    //   .mag_filt_window = 0,
    //   .temp_filt_window = 4,
    //   .pres_filt_window = 0,
    //   .antenna_offset_m = Vector3f::Zero(),
    //   .antenna_baseline_m = Vector3f::Zero(),
    //   .baseline_uncertainty_m = Vector3f::Zero(),
    //   .rotation = Matrix3f::Identity()
    // },
    // .ams5915_static_pres = {
    //   .addr = 0x10,
    //   .transducer = AMS5915_1200_B
    // },
    // .ams5915_diff_pres = {
    //   .addr = 0x11,
    //   .transducer = AMS5915_0020_D
    // },
    .gnss_uart3 = {
      .baud = 921600
    },
    // .gnss_uart4 = {
    //   .baud = 921600
    // }
  },
  .airdata = {
    .static_pres_source = AIR_DATA_STATIC_PRES_BME280,
    .static_pres_cutoff_hz = 5,
    .diff_pres_cutoff_hz = 5
  },
  .bfs_ekf = {
    .imu_source = EKF_IMU_MPU9250,
    .gnss_source_prim = EKF_GNSS_UBLOX3,
    .accel_cutoff_hz = 10,
    .gyro_cutoff_hz = 10,
    .mag_cutoff_hz = 10
  },
  .telem = {
    .aircraft_type = FIXED_WING,
    .imu_source = TELEM_IMU_MPU9250,
    .static_pres_source = TELEM_STATIC_PRES_BME280,
    .gnss_source = TELEM_GNSS_UBLOX3,
    .nav_source = TELEM_NAV_BFS_EKF,
    .bus = &Serial5,
    .rtk_uart = &Serial3,
    .baud = 57600
  }
};
