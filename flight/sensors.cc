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
/* Whether pitot static is installed */
bool pitot_static_installed_;
/* Sensors */
bfs::SbusRx inceptor;
bfs::Mpu9250 imu;
bfs::Ublox gnss;
bfs::Bme280 fmu_static_pres;
bfs::Ams5915 pitot_static_pres;
bfs::Ams5915 pitot_diff_pres;
}  // namespace

void SensorsInit(const SensorConfig &cfg) {
  pitot_static_installed_ = cfg.pitot_static_installed;
  MsgInfo("Intializing sensors...");
  /* Initialize IMU */
  if (!imu.Init(cfg.imu)) {
    MsgError("Unable to initialize IMU.");
  }
  /* Initialize GNSS */
  if (!gnss.Init(cfg.gnss)) {
    MsgError("Unable to initialize GNSS.");
  }
  /* Initialize pressure transducers */
  if (pitot_static_installed_) {
    if (!pitot_static_pres.Init(cfg.static_pres)) {
      MsgError("Unable to initialize static pressure sensor.");
    }
    if (!pitot_diff_pres.Init(cfg.diff_pres)) {
      MsgError("Unable to initialize differential pressure sensor.");
    }
  } else {
    if (!fmu_static_pres.Init(cfg.static_pres)) {
      MsgError("Unable to initialize static pressure sensor.");
    }
  }
  MsgInfo("done.\n");
  MsgInfo("Initializing inceptors...");
  while (!inceptor.Init(cfg.inceptor)) {}
  MsgInfo("done.\n");
}
void SensorsRead(SensorData * const data) {
  if (!data) {return;}
  /* Read inceptors */
  inceptor.Read(&data->inceptor);
  /* Read IMU */
  if (!imu.Read(&data->imu)) {
    MsgWarning("Unable to read IMU data.\n");
  }
  /* Read GNSS */
  gnss.Read(&data->gnss);
  /* Set whether pitot static is installed */
  data->pitot_static_installed = pitot_static_installed_;
  /* Read pressure transducers */
  if (pitot_static_installed_) {
    if (!pitot_static_pres.Read(&data->static_pres)) {
      MsgError("Unable to read pitot static pressure data.\n");
    }
    if (!pitot_diff_pres.Read(&data->diff_pres)) {
      MsgError("Unable to read pitot diff pressure data.\n");
    }
  } else {
    if (!fmu_static_pres.Read(&data->static_pres)) {
      MsgError("Unable to read FMU static pressure data.\n");
    }
  }
}
