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
        .ch = 6,
        .num_coef = 2,
        .poly_coef = {0.0012202562538133f, -1.20988407565589f}
      },
      .mode0 = {
        .ch = 4,
        .num_coef = 2,
        .poly_coef = {-0.0012202562538133f, 2.3f}
      },
      .mode1 = {
        .ch = 5,
        .num_coef = 2,
        .poly_coef = {-0.0012202562538133f, 2.3f}
      },
      .throttle = {
        .ch = 0,
        .num_coef = 2,
        .poly_coef = {0.00061013f, -0.10494204f}
      },
      .pitch = {
        .ch = 2,
        .num_coef = 2,
        .poly_coef = {0.0012203f, -1.2098841f}
      },
      .roll = {
        .ch = 1,
        .num_coef = 2,
        .poly_coef = {0.0012203f, -1.2098841f}
      },
      .yaw = {
        .ch = 3,
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
      .rotation = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
    },
    .gnss = {
      .sampling_period_ms = 100,  // 10 Hz
      .baud = 921600,
      .bus = &Serial3
    },
    .static_pres = {
      .dev = 0x10,
      .transducer = bfs::AMS5915_1200_B,
      .sampling_period_ms = FRAME_PERIOD_MS,
      .bus = &PRES_I2C_BUS
    },
    .diff_pres = {
      .dev = 0x11,
      .transducer = bfs::AMS5915_0010_D,
      .sampling_period_ms = FRAME_PERIOD_MS,
      .bus = &PRES_I2C_BUS
    }
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
      .effectors = {
        {
          /* Elevator */
          .type = bfs::SERVO,
          .ch = 2,
          .min = bfs::deg2rad(-20.0f),
          .max = bfs::deg2rad(20.0f),
          .failsafe = 0,
          .num_coef = 4,
          .poly_coef = {1144.32190165984f, 167.225360182927f,
                        1558.74885501875f, 1026.6382652891f}
        },
        {
          /* Rudder */
          .type = bfs::SERVO,
          .ch = 3,
          .min = bfs::deg2rad(-20.0f),
          .max = bfs::deg2rad(20.0f),
          .failsafe = 0,
          .num_coef = 4,
          .poly_coef = {-930.322085258545f, 148.612787752928f,
                        -1476.11502090935f, 1006.01614399429f}
        },
        {
          /* Left Aileron */
          .type = bfs::SERVO,
          .ch = 4,
          .min = bfs::deg2rad(-20.0f),
          .max = bfs::deg2rad(20.0f),
          .failsafe = 0,
          .num_coef = 4,
          .poly_coef = {1097.27825386315f, 173.191562145482f,
                        1642.60230905023f, 1054.30469578325f}
        },
        {
          /* Right Aileron */
          .type = bfs::SERVO,
          .ch = 5,
          .min = bfs::deg2rad(-20.0f),
          .max = bfs::deg2rad(20.0f),
          .failsafe = 0,
          .num_coef = 4,
          .poly_coef = {930.582953971947f, 132.665450728095f,
                        1620.14796233637f, 1011.10438715363f}
        }
      }
    },
    .pwm = {
      .hw = PWM_PINS,
      .effectors = {
        {
          /* ESC */
          .type = bfs::MOTOR,
          .ch = 0,
          .min = 0,
          .max = 1,
          .failsafe = 0,
          .num_coef = 2,
          .poly_coef = {1000.0f, 1000.0f}
        }
      }
    }
  },
  .telem = {
    .aircraft_type = bfs::FIXED_WING,
    .bus = &Serial7,
    .baud = 57600
  }
};
