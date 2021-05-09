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
#include "flight/global_defs.h"

/* Debug */
bool DEBUG = true;
/* Aircraft config */
AircraftConfig config = {
  .sensor = {
    .pitot_static_installed = true,
    .inceptor = {
      .hw = &SBUS_UART,
      .throttle_en = {

      },
      .mode0 = {

      },
      .mode1 = {

      },
      .mode2 = {

      },
      .mode3 = {

      },
      .throttle = {

      },
      .pitch = {

      },
      .roll = {

      },
      .yaw = {

      }
    },
    .imu = {
      .frame_rate = FRAME_RATE_HZ,
      .dev = IMU_CS,
      .bus = &IMU_SPI_BUS,
      .accel_bias_mps2 = Eigen::Vector3f::Zero(),
      .mag_bias_ut = Eigen::Vector3f::Zero(),
      .accel_scale = Eigen::Matrix3f::Identity(),
      .mag_scale = Eigen::Matrix3f::Identity(),
      .rotation = Eigen::Matrix3f::Identity() * IMU_ROTATION
    },
    .gnss = {
      .sampling_period_ms = 200,  // 5 Hz
      .baud = 921600,
      .bus = &Serial3
    },
    .fmu_static_pres = {
      .dev = PRES_CS,
      .sampling_period_ms = FRAME_PERIOD_MS,
      .bus = &PRES_SPI_BUS
    },
    .pitot_static_pres = {
      .dev = 0x10,
      .transducer = bfs::AMS5915_1200_B,
      .sampling_period_ms = FRAME_PERIOD_MS,
      .bus = &PRES_I2C_BUS
    },
    .pitot_diff_pres = {
      .dev = 0x11,
      .transducer = bfs::AMS5915_0010_D,
      .sampling_period_ms = FRAME_PERIOD_MS,
      .bus = &PRES_I2C_BUS
    }
  },
  .effector = {
    .sbus = {
      .hw = &SBUS_UART,
      .effectors = {

      }
    },
    .pwm = {
      .hw = PWM_PINS,
      .effectors = {
        
      }
    }
  },
  .telem = {
    .aircraft_type = bfs::FIXED_WING,
    .bus = &Serial4,
    .baud = 57600
  }
};
