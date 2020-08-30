/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "fixed_wing/effector.h"
#include "fixed_wing/print_msg.h"
#include "fixed_wing/hardware_defs.h"
#include "fixed_wing/global_defs.h"
#include "sbus/sbus.h"
#include "pwm/pwm.h"

namespace effector {
namespace {
/* SBUS object */
actuators::Sbus sbus_(&SBUS_UART);
/* PWM object */
actuators::Pwm<NUM_PWM_PINS> pwm_(PWM_PINS);
}  // anonymous

void Init() {
  print::Info("Initializing effectors...");
  sbus_.Begin();
  pwm_.Begin();
  print::Info("done.\n");
}
void Cmd(const EffectorCmds &ref) {
  sbus_.tx_channels(ref.sbus);
  pwm_.tx_channels(ref.pwm);
}
void Write() {
  sbus_.Write();
  pwm_.Write();
}

}  // namespace inceptor
