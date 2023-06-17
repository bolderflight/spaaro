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
#include "hardware_defs.h"
#include "global_defs.h"

/* Debug */
bool DEBUG = false;
/* Aircraft config */
AircraftConfig config = {
  .sensor = {
    .fmu = {
      .dlpf_hz = DLPF_BANDWIDTH_41HZ,
      .accel_bias_mps = {0, 0, 0},
      .mag_bias_ut = {0, 0, 0},
      .accel_scale = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
      .mag_scale = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
      .rotation = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
    },
    .ext_gnss1 = {
      .baud = 921600
    },
    .ext_gnss2 = {
      .baud = 921600
    },
    .power_module = {
      .volts_per_volt = 18.95f,
      .amps_per_volt = 125.65f
    }
  },
  .bfs_ins = {
    .imu_source = INS_IMU_FMU,
    .mag_source = INS_MAG_FMU,
    .gnss_source = INS_GNSS_EXT_GNSS2,
    .accel_cutoff_hz = 40.0f,
    .gyro_cutoff_hz = 40.0f,
    .antenna_baseline_m = (Eigen::Vector3f() << 0.0f, -0.62f, 0.0f).finished()
  },
  .telem = {
    .baud = 57600,
    .aircraft_type = bfs::MULTIROTOR
  }
};
