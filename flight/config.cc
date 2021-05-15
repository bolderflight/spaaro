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
    .pitot_static_installed = false,
    .inceptor = {
      .hw = &SBUS_UART,
      .throttle_en = {
        .ch = 0,
        .num_coef = 2,
        .poly_coef = {0.00061013f, -0.10494204f}
      },
      .mode0 = {
        .ch = 1,
        .num_coef = 2,
        .poly_coef = {0.0012203f, -0.2098841f}
      },
      .mode1 = {
        .ch = 2,
        .num_coef = 2,
        .poly_coef = {0.0012203f, -0.2098841f}
      },
      .throttle= {
        .ch = 3,
        .num_coef = 2,
        .poly_coef = {0.00061013f, -0.10494204f}
      },
      .pitch = {
        .ch = 4,
        .num_coef = 2,
        .poly_coef = {0.0012203f, -1.2098841f}
      },
      .roll = {
        .ch = 5,
        .num_coef = 2,
        .poly_coef = {0.0012203f, -1.2098841f}
      },
      .yaw = {
        .ch = 6,
        .num_coef = 2,
        .poly_coef = {0.0012203f, -1.2098841f}
      }
    },
    .imu = {
      .dev = IMU_CS,
      .frame_rate = FRAME_RATE_HZ,
      .bus = &IMU_SPI_BUS,
      .accel_bias_mps2 = {0, 0, 0},
      .mag_bias_ut = {0, 0, 0},
      .accel_scale = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
      .mag_scale = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
      .rotation = {{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}
    },
    .gnss = {
      .sampling_period_ms = 200,  // 5 Hz
      .baud = 921600,
      .bus = &Serial3
    },
    .static_pres = {
      .dev = PRES_CS,
      .sampling_period_ms = FRAME_PERIOD_MS,
      .bus = &PRES_SPI_BUS
    },
  },
  .nav = {
    .accel_cutoff_hz = 20,
    .gyro_cutoff_hz = 20,
    .mag_cutoff_hz = 10,
    .static_pres_cutoff_hz = 10,
    .diff_pres_cutoff_hz = 10
  },
  .effector = {
    .sbus = {
      .hw = &SBUS_UART,
      .effectors = {}
    },
    .pwm = {
      .hw = PWM_PINS,
      .effectors = {}
    }
  },
  .telem = {
    .aircraft_type = bfs::MULTIROTOR,
    .bus = &Serial4,
    .baud = 57600
  }
};
