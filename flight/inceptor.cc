/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "flight/inceptor.h"
#include "flight/print_msg.h"
#include "flight/hardware_defs.h"
#include "flight/global_defs.h"
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
