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
#include "ainstein_usd1.h"
#include "flight/msg.h"
#include "drivers/spaaro-ainstein-usd1.h"

void SpaaroAinsteinUsd1::Init(const RadAltConfig &cfg) {
  if (cfg.device != RAD_ALT_NONE) {
    if (!alt_.Begin()) {
      MsgError("Unable to establish communication with RADAR altimeter");
    } else {
      installed_ = true;
    }
  } else {
    installed_ = false;
  }
}

void SpaaroAinsteinUsd1::Read(RadAltData * const data) {
  data->installed = installed_;
  if (data->installed) {
    data->new_data = alt_.Read();
    if (data->new_data) {
      t_healthy_ms_ = 0;
      data->alt_m = alt_.alt_m();
      data->snr = alt_.snr();
    }
    data->healthy = (t_healthy_ms_ < 10 * UPDATE_PERIOD_MS_);
  }
}
