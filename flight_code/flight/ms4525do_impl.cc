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

#include "flight/ms4525do_impl.h"
#include "flight/global_defs.h"
#include "flight/hardware_defs.h"
#include "flight/msg.h"
#include "ms4525do.h"  // NOLINT
#include "statistics.h"  // NOLINT

namespace {
/* MS4525DO object */
Ms4525do ms4525;
/* Config */
Ms4525doConfig cfg;
/* Remove diff pres bias */
static constexpr int32_t INIT_TIME_MS = 5000;
float diff_pres_bias_pa;
RunningStats<float> dp;
/* Healthy */
elapsedMillis time_ms;
bool status;
bool healthy;
}  // namespace

void Ms4525doInit(const Ms4525doConfig &config) {
  cfg = config;
  if (cfg.addr != 0) {
    ms4525.Config(&I2C_BUS, cfg.addr, cfg.max_pres, cfg.min_pres,
                  cfg.output_type);
    if (!ms4525.Begin()) {
      MsgError("unable to communicate with MS4525DO pressure transducer");
    }
    time_ms = 0;
    while (time_ms < INIT_TIME_MS) {
      if (ms4525.Read()) {
        dp.Update(ms4525.pres_pa());
      }
      delay(1);
    }
    diff_pres_bias_pa = -dp.mean();
  }
}
void Ms4525doRead() {
  if (cfg.addr != 0) {
    status = ms4525.Read();
    if (status) {
      time_ms = 0;
    } else {
      MsgWarning("error reading MS4525DO pressure transducer");
    }
    healthy = (time_ms < HEALTHY_TIMEOUT_MS);
  }
}
void Ms4525doPresData(PresData * const data) {
  if (data) {
    if (cfg.addr != 0) {
      data->installed = true;
      data->new_data = status;
      data->healthy = healthy;
      if (data->new_data) {
        data->die_temp_c = ms4525.die_temp_c();
        data->pres_pa = std::abs(ms4525.pres_pa() + diff_pres_bias_pa);
      }
    } else {
      data->installed = false;
    }
  }
}

