/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "flight/status.h"
#include "flight/print_msg.h"
#include "flight/hardware_defs.h"
#include "flight/global_defs.h"

namespace status {
namespace {
/* Convert analog read values to voltage */
static constexpr float VOLTAGE_RANGE_ = 3.3;
static constexpr float COUNT_RANGE_ = powf(2.0f, ANALOG_RESOLUTION_BITS) - 1.0f;
}  // anonymous

void Init() {
  print::Info("Initializing voltage measurements...");
  analogReadResolution(ANALOG_RESOLUTION_BITS);
  print::Info("done.\n");
}
void Read(StatusData *ptr) {
  ptr->input_voltage = static_cast<float>(analogRead(INPUT_VOLTAGE_PIN)) * VOLTAGE_RANGE_ / COUNT_RANGE_ * INPUT_VOLTAGE_SCALE;
  ptr->regulated_voltage = static_cast<float>(analogRead(REGULATED_VOLTAGE_PIN)) * VOLTAGE_RANGE_ / COUNT_RANGE_ * REGULATED_VOLTAGE_SCALE;
  ptr->sbus_voltage = static_cast<float>(analogRead(SBUS_VOLTAGE_PIN)) * VOLTAGE_RANGE_ / COUNT_RANGE_ * SBUS_VOLTAGE_SCALE;
  ptr->pwm_voltage = static_cast<float>(analogRead(PWM_VOLTAGE_PIN)) * VOLTAGE_RANGE_ / COUNT_RANGE_ * PWM_VOLTAGE_SCALE;
}

}  // namespace status
