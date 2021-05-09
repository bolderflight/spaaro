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

#include "flight/sensors.h"
#include "flight/global_defs.h"
#include "flight/config.h"
#include "flight/msg.h"

namespace {
bool pitot_static_installed_;
}  // namespace

void SensorsInit(const SensorConfig &cfg, Sensors * const obj) {
  if (!obj) {return;}
  pitot_static_installed_ = cfg.pitot_static_installed;
  MsgInfo("Intializing sensors...");
  if (!obj->inceptor.Init(cfg.inceptor)) {
    MsgError("Unable to initialize inceptor.");
  }
  if (!obj->imu.Init(cfg.imu)) {
    MsgError("Unable to initialize IMU.");
  }
  if (!obj->gnss.Init(cfg.gnss)) {
    MsgError("Unable to initialize GNSS.");
  }
  if (cfg.pitot_static_installed) {
    if (!obj->pitot_static_pres.Init(cfg.pitot_static_pres)) {
      MsgError("Unable to initialize pitot static pressure sensor.");
    }
    if (!obj->pitot_diff_pres.Init(cfg.pitot_diff_pres)) {
      MsgError("Unable to initialize pitot differential pressure sensor.");
    }
  } else {
    if (!obj->fmu_static_pres.Init(cfg.fmu_static_pres)) {
      MsgError("Unable to initialize FMU static pressure sensor.");
    }
  }
  MsgInfo("done.\n");
}
void SensorsRead(SensorData * const data, Sensors * const obj) {
  if ((!data) || (!obj)) {return;}
  obj->inceptor.Read(&data->inceptor);
  if (!obj->imu.Read(&data->imu)) {
    MsgWarning("Unable to read IMU data.\n");
  }
  obj->gnss.Read(&data->gnss);
  data->pitot_static_installed = pitot_static_installed_;
  if (pitot_static_installed_) {
    if (!obj->pitot_static_pres.Read(&data->pitot_static_pres)) {
      MsgError("Unable to read pitot static pressure data.\n");
    }
    if (!obj->pitot_diff_pres.Read(&data->pitot_diff_pres)) {
      MsgError("Unable to read pitot diff pressure data.\n");
    }
  } else {
    if (!obj->fmu_static_pres.Read(&data->fmu_static_pres)) {
      MsgError("Unable to read FMU static pressure data.\n");
    }
  }
}
