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
#include "flight/adc.h"
#include "filter.h"
#include "airdata.h"

namespace {
AdcConfig cfg_;
bool adc_initialized_ = false;
bool diff_source_avail_ = false;
float init_pres_alt_m_;
PresData *static_pres_, *diff_pres_;
bfs::Iir<float> static_pres_dlpf_, diff_pres_dlpf_;
}  // namespace

void AdcInit(const AdcConfig &cfg) {
  cfg_ = cfg;
}

void AdcRun(SensorData &sensor, AdcData * const data) {
  if (!adc_initialized_) {
    /* Setup the static pressure source */
    switch (cfg_.static_pres_source) {
      case ADC_STATIC_PRES_FMU: {
        if (sensor.fmu_static_pres.installed) {
          static_pres_ = &sensor.fmu_static_pres;
        } else {
          MsgError("ADC static pressure source set to FMU, which is not installed");
        }
        break;
      }
      case ADC_STATIC_PRES_EXT_PRES1: {
        if (sensor.ext_pres1.installed) {
          if (sensor.ext_pres1.is_static_pres) {
            static_pres_ = &sensor.ext_pres1;
          } else {
            MsgError("ADC static pressure source set to ext pres1, which is not a static pressure sensor");
          }
        } else {
          MsgError("ADC static pressure source set to ext pres1, which is not installed");
        }
        break;
      }
      case ADC_STATIC_PRES_EXT_PRES2: {
        if (sensor.ext_pres2.installed) {
          if (sensor.ext_pres2.is_static_pres) {
            static_pres_ = &sensor.ext_pres2;
          } else {
            MsgError("ADC static pressure source set to ext pres2, which is not a static pressure sensor");
          }
        } else {
          MsgError("ADC static pressure source set to ext pres2, which is not installed");
        }
        break;
      }
      case ADC_STATIC_PRES_EXT_PRES3: {
        if (sensor.ext_pres3.installed) {
          if (sensor.ext_pres3.is_static_pres) {
            static_pres_ = &sensor.ext_pres3;
          } else {
            MsgError("ADC static pressure source set to ext pres3, which is not a static pressure sensor");
          }
        } else {
          MsgError("ADC static pressure source set to ext pres3, which is not installed");
        }
        break;
      }
      case ADC_STATIC_PRES_EXT_PRES4: {
        if (sensor.ext_pres4.installed) {
          if (sensor.ext_pres4.is_static_pres) {
            static_pres_ = &sensor.ext_pres4;
          } else {
            MsgError("ADC static pressure source set to ext pres4, which is not a static pressure sensor");
          }
        } else {
          MsgError("ADC static pressure source set to ext pres4, which is not installed");
        }
        break;
      }
    }
    /* Setup diff pressure source */
    switch (cfg_.diff_pres_source) {
      case ADC_DIFF_PRES_NONE: {
        break;
      }
      case ADC_DIFF_PRES_EXT_PRES1: {
        if (sensor.ext_pres1.installed) {
          if (!sensor.ext_pres1.is_static_pres) {
            diff_pres_ = &sensor.ext_pres1;
            diff_source_avail_ = true;
          } else {
            MsgError("ADC diff pressure source set to ext pres1, which is not a diff pressure sensor");
          }
        } else {
          MsgError("ADC diff pressure source set to ext pres1, which is not installed");
        }
        break;
      }
      case ADC_DIFF_PRES_EXT_PRES2: {
        if (sensor.ext_pres2.installed) {
          if (!sensor.ext_pres2.is_static_pres) {
            diff_pres_ = &sensor.ext_pres2;
            diff_source_avail_ = true;
          } else {
            MsgError("ADC diff pressure source set to ext pres2, which is not a diff pressure sensor");
          }
        } else {
          MsgError("ADC diff pressure source set to ext pres2, which is not installed");
        }
        break;
      }
      case ADC_DIFF_PRES_EXT_PRES3: {
        if (sensor.ext_pres3.installed) {
          if (!sensor.ext_pres3.is_static_pres) {
            diff_pres_ = &sensor.ext_pres3;
            diff_source_avail_ = true;
          } else {
            MsgError("ADC diff pressure source set to ext pres3, which is not a diff pressure sensor");
          }
        } else {
          MsgError("ADC diff pressure source set to ext pres3, which is not installed");
        }
        break;
      }
      case ADC_DIFF_PRES_EXT_PRES4: {
        if (sensor.ext_pres4.installed) {
          if (!sensor.ext_pres4.is_static_pres) {
            diff_pres_ = &sensor.ext_pres4;
            diff_source_avail_ = true;
          } else {
            MsgError("ADC diff pressure source set to ext pres4, which is not a diff pressure sensor");
          }
        } else {
          MsgError("ADC diff pressure source set to ext pres4, which is not installed");
        }
        break;
      }
    }
    /* Setup the low pass filters */
    if (diff_source_avail_) {
      diff_pres_dlpf_.Init(cfg_.diff_pres_cutoff_hz, FRAME_RATE_HZ);
    }
    if (static_pres_->new_data) {
      static_pres_dlpf_.Init(cfg_.static_pres_cutoff_hz, FRAME_RATE_HZ,
                             static_pres_->pres_pa);
      init_pres_alt_m_ = bfs::PressureAltitude_m(static_pres_->pres_pa);
      adc_initialized_ = true;
    }
  } else {
    if (static_pres_->new_data) {
      data->static_pres_pa = static_pres_dlpf_.Filter(static_pres_->pres_pa);
    }
    data->pres_alt_m = bfs::PressureAltitude_m(data->static_pres_pa);
    data->rel_alt_m = data->pres_alt_m - init_pres_alt_m_;
    if (diff_source_avail_) {
      if (diff_pres_->new_data) {
        data->diff_pres_pa = diff_pres_dlpf_.Filter(diff_pres_->pres_pa);
      }
      data->ias_mps = bfs::Ias_mps(data->diff_pres_pa);
    }
  }
}
