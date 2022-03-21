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

#include "flight/ams5915_impl.h"
#include "flight/global_defs.h"
#include "flight/hardware_defs.h"
#include "flight/msg.h"
#include "ams5915.h"
#include "statistics.h"

namespace {
/* AMS-5915 objects */
Ams5915 ams5915_static_pres, ams5915_diff_pres;
/* Configs */
Ams5915Config cfg_static_pres, cfg_diff_pres;
/* Remove diff pres bias */
elapsedMillis time_ms;
static constexpr int32_t INIT_TIME_MS = 5000;
float diff_pres_bias_pa;
RunningStats<float> dp;
/* Interim data */
bool sp_status, dp_status;
/* Healthy */
elapsedMillis sp_time_ms, dp_time_ms;
bool sp_healthy, dp_healthy;
}

void Ams5915Init(const Ams5915Config &static_pres,
                 const Ams5915Config &diff_pres) {
  cfg_static_pres = static_pres;
  cfg_diff_pres = diff_pres;
  if (cfg_static_pres.transducer != AMS5915_NONE) {
    ams5915_static_pres.Config(&I2C_BUS, cfg_static_pres.addr,
      static_cast<Ams5915::Transducer>(cfg_static_pres.transducer));
    if (!ams5915_static_pres.Begin()) {
      MsgError("unable to communicate with AMS5915 static pressure transducer");
    }
  }
  if (cfg_diff_pres.transducer != AMS5915_NONE) {
    ams5915_diff_pres.Config(&I2C_BUS, cfg_diff_pres.addr,
      static_cast<Ams5915::Transducer>(cfg_diff_pres.transducer));
    if (!ams5915_diff_pres.Begin()) {
      MsgError("unable to communicate with AMS5915 differential pressure \
                transducer");
    }
    dp_time_ms = 0;
    while (dp_time_ms < INIT_TIME_MS) {
      if (ams5915_diff_pres.Read()) {
        dp.Update(ams5915_diff_pres.pres_pa());
      }
      delay(1);
    }
    diff_pres_bias_pa = -dp.mean();
  }
}

void Ams5915Read() {
  if (cfg_static_pres.transducer != AMS5915_NONE) {
    sp_status = ams5915_static_pres.Read();
    if (sp_status) {
      sp_time_ms = 0;
    } else {
      MsgWarning("error reading AMS5915 static pressure transducer");
    }
    sp_healthy = (sp_time_ms < HEALTHY_TIMEOUT_MS);
  }
  if (cfg_diff_pres.transducer != AMS5915_NONE) {
    dp_status = ams5915_diff_pres.Read();
    if (dp_status) {
      dp_time_ms = 0;
    } else {
      MsgWarning("error reading AMS5915 differential pressure transducer");
    }
    dp_healthy = (dp_time_ms < HEALTHY_TIMEOUT_MS);
  }
}

void Ams5915PresData(PresData * const static_pres, PresData * const diff_pres) {
  if (cfg_static_pres.transducer != AMS5915_NONE) {
    static_pres->installed = true;
    static_pres->new_data = sp_status;
    static_pres->healthy = sp_healthy;
    if (static_pres->new_data) {
      static_pres->die_temp_c = ams5915_static_pres.die_temp_c();
      static_pres->pres_pa = ams5915_static_pres.pres_pa();
    }
  } else {
    static_pres->installed = false;
  }
  if (cfg_diff_pres.transducer != AMS5915_NONE) {
    diff_pres->installed = true;
    diff_pres->new_data = dp_status;
    diff_pres->healthy = dp_healthy;
    if (diff_pres->new_data) {
      diff_pres->die_temp_c = ams5915_diff_pres.die_temp_c();
      diff_pres->pres_pa = ams5915_diff_pres.pres_pa() + diff_pres_bias_pa;
    }
  } else {
    diff_pres->installed = false;
  }
}
