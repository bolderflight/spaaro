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
#include "drivers/spaaro-ams5915.h"
#include "statistics.h"

void SpaaroAms5915::Init(const PresConfig &cfg) {
  if (cfg.device != PRES_NONE) {
    if ((cfg.device == PRES_AMS5915_1000_A) ||
        (cfg.device == PRES_AMS5915_1200_B)) {
      is_static_pres_ = true;
    } else {
      is_static_pres_ = false;
    }
    pres_.Config(i2c_, cfg.addr,
                static_cast<bfs::Ams5915::Transducer>(cfg.device));
    if (!pres_.Begin()) {
      MsgError("Unable to establish communication with external pressure transducer");
    }
    transducer_ = cfg.device;
    installed_ = true;
  } else {
    installed_ = false;
  }
}

void SpaaroAms5915::Cal() {
  if ((transducer_ == PRES_AMS5915_1000_A) ||
      (transducer_ == PRES_AMS5915_1200_B)) {return;}
  if (installed_) {
    if (pres_.Read()) {
      bias_.Update(pres_.pres_pa());
    }
  }
}

void SpaaroAms5915::Read(PresData * const data) {
  data->installed = installed_;
  data->is_static_pres = is_static_pres_;
  if (data->installed) {
    data->new_data = pres_.Read();
    if (data->new_data) {
      t_healthy_ms_ = 0;
      data->pres_pa = pres_.pres_pa() - bias_.mean();
      data->die_temp_c = pres_.die_temp_c();
    }
    data->healthy = (t_healthy_ms_ < 10 * FRAME_PERIOD_MS);
  }
}
