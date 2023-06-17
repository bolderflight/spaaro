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
#include "flight/msg.h"
#include "drivers/spaaro-lis3mdl.h"
#include "eigen.h"
#include "lis3mdl.h"

void SpaaroLis3mdl::Init(const MagConfig &cfg) {
  if (cfg.device != EXT_MAG_NONE) {
    if (cfg.device == EXT_MAG_PRIM) {
      mag_.Config(&I2C_BUS, bfs::Lis3mdl::I2C_ADDR_PRIM);
    } else {
      mag_.Config(&I2C_BUS, bfs::Lis3mdl::I2C_ADDR_SEC);
    }
    if (!mag_.Begin()) {
      MsgError("Unable to establish communication with external mag");
    }
    if (!mag_.ConfigRange(Lis3mdl::RANGE_16GS)) {
      MsgError("Unable to configure external mag range");
    }
    if (!mag_.ConfigOdr(Lis3mdl::ODR_155HZ)) {
      MsgError("Unable to configure external mag data rate");
    }
    /* Save config */
    for (int8_t m = 0; m < 3; m++) {
      mag_bias_ut_[m] = cfg.mag_bias_ut[m];
      for (int8_t n = 0; n < 3; n++) {
        mag_scale_(m ,n) = cfg.mag_scale[m][n];
        rotation_(m ,n) = cfg.rotation[m][n];
      }
    }
    installed_ = true;
  } else {
    installed_ = false;
  }
}

void SpaaroLis3mdl::Read(MagData * const data) {
  data->installed = installed_;
  if (data->installed) {
    data->new_data = mag_.Read();
    if (data->new_data) {
      t_healthy_ms_ = 0;
      mag_ut_[0] = mag_.mag_x_ut();
      mag_ut_[1] = mag_.mag_y_ut();
      mag_ut_[2] = mag_.mag_z_ut();
      mag_ut_ = mag_scale_ * rotation_ * mag_ut_ + mag_bias_ut_;
      data->mag_ut[0] = mag_ut_[0];
      data->mag_ut[1] = mag_ut_[1];
      data->mag_ut[2] = mag_ut_[2];
    }
    data->healthy = (t_healthy_ms_ < 10 * FRAME_PERIOD_MS);
  }
}
