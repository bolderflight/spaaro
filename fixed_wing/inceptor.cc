/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "fixed_wing/inceptor.h"
#include "fixed_wing/print_msg.h"
#include "fixed_wing/hardware_defs.h"
#include "fixed_wing/global_defs.h"
#include "sbus/sbus.h"

namespace inceptor {
namespace {
/* SBUS object */
sensors::Sbus sbus_(&SBUS_UART);
}  // anonymous

void Init() {
  print::Info("Initializing inceptors...");
  sbus_.Begin();
  print::Info("done.\n");
}
void Read(InceptorData *ptr) {
  if (sbus_.Read()) {
    ptr->cmds = sbus_.rx_channels();
  }
}

}  // namespace inceptor
