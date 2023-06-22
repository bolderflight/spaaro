/*
* Tuan Luong    
* tdluong@crimson.ua.edu
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
#include "matek3901.h"
#include "flight/msg.h"
#include "drivers/spaaro-matek3901.h"

void SpaaroMatek3901::Init(const OpFlowConfig &cfg) {
  if (cfg.device != OPFLOW_NONE) {
    if (!opflow_.Begin()) {
      MsgError("Unable to establish communication with optical flow");
    } else {
      installed_ = true;
    }
  } else {
    installed_ = false;
  }
}

void SpaaroMatek3901::Read(OpFlowData * const data) {
  data->installed = installed_;
  if (data->installed) {
    data->new_data = opflow_.Read();
    if (data->new_data) {
      t_healthy_ms_ = 0;
      data->sur_qual = opflow_.sur_qual();
      data->range_qual = opflow_.range_qual();
      data->mot_x = opflow_.mot_x();
      data->mot_y = opflow_.mot_y();
      data->range_mm = opflow_.range_mm();
    }
    data->healthy = (t_healthy_ms_ < 10 * UPDATE_PERIOD_MS_);
  }
}
