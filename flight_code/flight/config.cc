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
    .sbus = {
      .installed = true
    },
    .mpu9250 = {

    },
    .vector_nav = {
      .device = VECTORNAV_VN300
    },
    .ams5915_static_pres = {
      .addr = 0x10,
      .transducer = AMS5915_1200_B
    },
    .ams5915_diff_pres = {
      .addr = 0x11,
      .transducer = AMS5915_0020_D
    },
    .gnss_uart3 = {
      .baud = 921600
    },
    .gnss_uart4 = {
      .baud = 921600
    }
  },
  .airdata = {
    .static_pres_source = AIR_DATA_STATIC_PRES_AMS5915,
    .static_pres_cutoff_hz = 5,
    .diff_pres_cutoff_hz = 5
  },
  .bfs_ekf = {

  },
  .telem = {
    .bus = &Serial4,
    .rtk_uart = &Serial3,
    .baud = 57600
  }
};
