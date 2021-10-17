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

#if defined(__FMU_R_V2__)

#include "flight/battery.h"
#include "flight/global_defs.h"
#include "flight/config.h"
#include "flight/msg.h"
#include "filter/filter.h"

namespace {
/* Battery config */
BatteryConfig cfg_;
/* Filter for the current */
bfs::Iir<float> current_ma;
}  // namespace

void BatteryInit(const BatteryConfig &cfg) {
  /* Copy the config */
  cfg_ = cfg;
  /* Initialize the filter */
  current_ma.Init(cfg.current_cutoff_hz, static_cast<float>(FRAME_RATE_HZ));
}
void BatteryRead(BatteryData * const data) {
  data->voltage_v = static_cast<float>(analogRead(BATTERY_VOLTAGE_PIN)) *
                    AIN_VOLTAGE_SCALE * cfg_.voltage_scale;
  data->current_ma = static_cast<float>(analogRead(BATTERY_CURRENT_PIN)) *
                     AIN_VOLTAGE_SCALE * cfg_.current_scale * 1000.0f;
  data->consumed_mah += data->current_ma * FRAME_PERIOD_MS / 1000.0f / 3600.0f;
  data->remaining_prcnt = (cfg_.capacity_mah - data->consumed_mah) /
                           cfg_.capacity_mah * 100.0f;
  data->remaining_time_s = (cfg_.capacity_mah - data->consumed_mah) /
                            current_ma.Filter(data->consumed_mah) * 60.0f;
}

#endif
