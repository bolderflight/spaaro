/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "flight/airdata.h"
#include "flight/print_msg.h"
#include "flight/hardware_defs.h"
#include "flight/global_defs.h"
#include "bme280/bme280.h"
#include "ams5915/ams5915.h"
#include "airdata/airdata.h"
#include "filter/filter.h"
#include "statistics/statistics.h"

namespace airdata {
namespace {
/* BME280 */
sensors::Bme280 fmu_static_press_(&FMU_PRESS_SPI_BUS, FMU_PRESS_CS);
/* AMS-5915 */
sensors::Ams5915 static_press_(&STATIC_PRESS_I2C_BUS, STATIC_PRESS_ADDR, STATIC_PRESS_TRANSDUCER);
sensors::Ams5915 diff_press_(&DIFF_PRESS_I2C_BUS, DIFF_PRESS_ADDR, DIFF_PRESS_TRANSDUCER);
/* Initialization time perion */
static constexpr float INIT_TIME_S_ = 10.0f;
/* Static and differential pressure statistics */
statistics::Running<float> static_press_stats_;
statistics::Running<float> diff_press_stats_;
/* Differential pressure bias */
float diff_press_bias_pa_;
/* Initial pressure altitude */
float init_press_alt_m_;
/* Static and differential pressure filters */
std::array<float, 1> b_ = {0.039205f};
std::array<float, 2> a_ = {1.0f, -0.96079f};
filters::DigitalFilter1D<float, b_.size(), a_.size()> static_press_filt_(b_, a_);
filters::DigitalFilter1D<float, b_.size(), a_.size()> diff_press_filt_(b_, a_);
}  // anonymous

void Init() {
  print::Info("Initializing pressure transducers...");
  if (!fmu_static_press_.Begin()) {
    print::Error("Unable to initialize communication with FMU integrated static pressure transducer.");
  }
  if (!static_press_.Begin()) {
    print::Error("Unable to initialize communication with static pressure transducer.");
  }
  if (!diff_press_.Begin()) {
    print::Error("Unable to initialize communication with differential pressure transducer.");
  }
  print::Info("done.\n");
  print::Info("Initializing airdata states...");
  elapsedMillis t_ms = 0;
  while (t_ms < INIT_TIME_S_ * 1000.0f) {
    if (static_press_.Read()) {
      static_press_stats_.Update(static_press_.pressure_pa());
      /* Warm up the static pressure filter */
      static_press_filt_.Filter(static_press_.pressure_pa());
    }
    if (diff_press_.Read()) {
      diff_press_stats_.Update(diff_press_.pressure_pa());
    }
    delay(FRAME_PERIOD_S * 1000.0f);
  }
  diff_press_bias_pa_ = diff_press_stats_.mean();
  init_press_alt_m_ = PressureAltitude_m(static_press_stats_.mean());
  print::Info("done.\n");
}
void Read(Airdata *ptr) {
  if (!ptr) {return;}
  if (fmu_static_press_.Read()) {
    ptr->fmu_static_press.press_pa = fmu_static_press_.pressure_pa();
    ptr->fmu_static_press.die_temp_c = fmu_static_press_.die_temperature_c();
  }
  if (static_press_.Read()) {
    /* Pressure transducer data */
    ptr->static_press.press_pa = static_press_.pressure_pa();
    ptr->static_press.die_temp_c = static_press_.die_temperature_c();
    /* Filtered static pressure */
    ptr->filt_static_press_pa = static_press_filt_.Filter(ptr->static_press.press_pa);
  }
  if (diff_press_.Read()) {
    /* Pressure transducer data */
    ptr->diff_press.press_pa = diff_press_.pressure_pa() - diff_press_bias_pa_;
    ptr->diff_press.die_temp_c = diff_press_.die_temperature_c();
    /* Filtered diff pressure */
    ptr->filt_diff_press_pa = diff_press_filt_.Filter(ptr->diff_press.press_pa);
  }
  /* Altitudes */
  ptr->press_alt_m = PressureAltitude_m(ptr->filt_static_press_pa);
  ptr->agl_alt_m = ptr->press_alt_m - init_press_alt_m_;
  /* Airspeeds */
  ptr->ias_mps = Ias_mps(ptr->filt_diff_press_pa);
  ptr->eas_mps = Eas_mps(ptr->filt_diff_press_pa, ptr->filt_static_press_pa);
}

}  // namespace airdata
