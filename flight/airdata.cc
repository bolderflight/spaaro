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

#include "flight/hardware_defs.h"
#include "flight/global_defs.h"
#include "flight/config.h"
#include "flight/msg.h"
#include "flight/airdata.h"
#include "bme280/bme280.h"
#include "ams5915/ams5915.h"
#include "statistics/statistics.h"
#include "filter/filter.h"

namespace {
#ifdef HAVE_PITOT_STATIC
/* AMS-5915 */
bfs::Ams5915 static_pres_(&STATIC_PRES_I2C_BUS, STATIC_PRES_ADDR,
                          STATIC_PRES_TRANSDUCER);
bfs::Ams5915 diff_pres_(&DIFF_PRES_I2C_BUS, DIFF_PRES_ADDR,
                        DIFF_PRES_TRANSDUCER);
/* Diff pres bias estimation */
bfs::RunningStats<float> diff_pres_stats_;
float diff_pres_bias_pa_ = 0.0f;
/* Diff pres filter */
bfs::DigitalFilter1D<float, DIFF_PRES_FILT_B.size(), DIFF_PRES_FILT_A.size()>
  diff_pres_filt(DIFF_PRES_FILT_B, DIFF_PRES_FILT_A);
#else
/* BME 280 */
bfs::Bme280 static_pres_(&FMU_PRES_SPI_BUS, FMU_PRES_CS);
#endif
/* Bias estimation and filter warmup */
elapsedMillis time_ms_;
static constexpr float INIT_TIME_S_ = 5.0f;
/* Static pres filter */
bfs::DigitalFilter1D<float, STATIC_PRES_FILT_B.size(),
                     STATIC_PRES_FILT_A.size()>
  static_pres_filt(STATIC_PRES_FILT_B, STATIC_PRES_FILT_A);
}  // namespace

void AirdataInit() {
  MsgInfo("Initializing pressure transducers...");
  #ifdef HAVE_PITOT_STATIC
    while (!static_pres_.Begin()) {}
    while (!diff_pres_.Begin()) {}
  #else
  if (!static_pres_.Begin()) {
    MsgError("Unable to init FMU integrated static pressure transducer.");
  }
  MsgInfo("done.\n");
  #endif
  MsgInfo("Removing bias and warming up filters...");
  time_ms_ = 0;
  while (time_ms_ < INIT_TIME_S_ * 1000.0f) {
    #ifdef HAVE_PITOT_STATIC
    if (diff_pres_.Read()) {
      diff_pres_stats_.Update(diff_pres_.pressure_pa());
    }
    #endif
    if (static_pres_.Read()) {
      static_pres_filt.Filter(static_pres_.pressure_pa());
    }
    delay(FRAME_PERIOD_S * 1000.0f);
  }
  #ifdef HAVE_PITOT_STATIC
  diff_pres_bias_pa_ = -diff_pres_stats_.mean();
  #endif
  MsgInfo("done.\n");
}
void AirdataRead(AirdataData * const ptr) {
  if (!ptr) {return;}
  ptr->static_pres.new_data = static_pres_.Read();
  if (ptr->static_pres.new_data) {
    ptr->static_pres.pres_pa = static_pres_.pressure_pa();
    ptr->static_pres.dlpf.pres_pa =
      static_pres_filt.Filter(ptr->static_pres.pres_pa);
    ptr->static_pres.die_temp_c = static_pres_.die_temperature_c();
  }
  #ifdef HAVE_PITOT_STATIC
  if (ptr->diff_pres.new_data) {
    ptr->diff_pres.pres_pa = diff_pres_.pressure_pa() + diff_pres_bias_pa_;
    ptr->diff_pres.dlpf.pres_pa =
      diff_pres_filt.Filter(ptr->diff_pres.pres_pa);
    ptr->diff_pres.die_temp_c = diff_pres_.die_temperature_c();
  }
  #endif
}
