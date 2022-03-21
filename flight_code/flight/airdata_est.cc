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

#include "flight/global_defs.h"
#include "flight/hardware_defs.h"
#include "flight/msg.h"
#include "filter.h"  // NOLINT
#include "airdata.h"  // NOLINT

namespace {
/* Filters */
Iir<float> static_pres_dlpf;
Iir<float> diff_pres_dlpf;
/* Configuration */
AirDataConfig config;
/* Flag whether airdata was initialized */
bool airdata_initialized = false;
bool static_pres_init = false, diff_pres_init = false;
}

void AirDataInit(const AirDataConfig &cfg) {
  config = cfg;
}

void AirDataEst(const SensorData &sens, AirData * const data) {
  if (!data) {return;}
  if (!airdata_initialized) {
    data->initialized = false;
    data->static_pres_pa = 0;
    data->diff_pres_pa = 0;
    data->alt_pres_m = 0;
    data->ias_mps = 0;
    switch (config.static_pres_source) {
      case AIR_DATA_STATIC_PRES_BME280: {
        if (!sens.bme280_static_pres.installed) {
          MsgError("BME280 selected as air data static pressure source,"
                   "but not installed.");
        }
        if ((!static_pres_init) && (sens.bme280_static_pres.new_data)) {
          static_pres_dlpf.Init(config.static_pres_cutoff_hz,
                                FRAME_RATE_HZ,
                                sens.bme280_static_pres.pres_pa);
          static_pres_init = true;
        }
        break;
      }
      case AIR_DATA_STATIC_PRES_AMS5915: {
        if (!sens.ams5915_static_pres.installed) {
          MsgError("AMS5915 selected as air data static pressure source,"
                   "but not installed.");
        }
        if ((!static_pres_init) && (sens.ams5915_static_pres.new_data)) {
          static_pres_dlpf.Init(config.static_pres_cutoff_hz,
                                FRAME_RATE_HZ,
                                sens.ams5915_static_pres.pres_pa);
          static_pres_init = true;
        }
        break;
      }
      case AIR_DATA_STATIC_PRES_VECTORNAV: {
        if (!sens.vector_nav_static_pres.installed) {
          MsgError("VectorNav selected as air data static pressure source,"
                   "but not installed.");
        }
        if ((!static_pres_init) && (sens.vector_nav_static_pres.new_data)) {
          static_pres_dlpf.Init(config.static_pres_cutoff_hz,
                                FRAME_RATE_HZ,
                                sens.vector_nav_static_pres.pres_pa);
          static_pres_init = true;
        }
        break;
      }
    }
    if (sens.ams5915_diff_pres.installed) {
      if ((!diff_pres_init) && (sens.ams5915_diff_pres.new_data)) {
        diff_pres_dlpf.Init(config.diff_pres_cutoff_hz,
                              FRAME_RATE_HZ,
                              sens.ams5915_diff_pres.pres_pa);
        diff_pres_init = true;
      }
    } else {
      diff_pres_init = true;
    }
    if ((static_pres_init) && (diff_pres_init)) {
      airdata_initialized = true;
    }
  } else {
    data->initialized = true;
    switch (config.static_pres_source) {
      case AIR_DATA_STATIC_PRES_BME280: {
        if (sens.bme280_static_pres.new_data) {
          data->static_pres_pa =
            static_pres_dlpf.Filter(sens.bme280_static_pres.pres_pa);
          data->alt_pres_m = PressureAltitude_m(data->static_pres_pa);
        }
        break;
      }
      case AIR_DATA_STATIC_PRES_AMS5915: {
        if (sens.ams5915_static_pres.new_data) {
          data->static_pres_pa =
            static_pres_dlpf.Filter(sens.ams5915_static_pres.pres_pa);
          data->alt_pres_m = PressureAltitude_m(data->static_pres_pa);
        }
        break;
      }
      case AIR_DATA_STATIC_PRES_VECTORNAV: {
        if (sens.vector_nav_static_pres.new_data) {
          data->static_pres_pa =
            static_pres_dlpf.Filter(sens.vector_nav_static_pres.pres_pa);
          data->alt_pres_m = PressureAltitude_m(data->static_pres_pa);
        }
        break;
      }
    }
    if (sens.ams5915_diff_pres.installed) {
      if (sens.ams5915_diff_pres.new_data) {
        data->diff_pres_pa =
          diff_pres_dlpf.Filter(sens.ams5915_diff_pres.pres_pa);
        data->ias_mps = Ias_mps(data->diff_pres_pa);
      }
    } else {
      data->diff_pres_pa = 0;
      data->ias_mps = 0;
    }
  }
}
