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

#include "flight/sbus_rx_impl.h"
#include "flight/global_defs.h"
#include "flight/hardware_defs.h"
#include "flight/msg.h"
#include "sbus.h"

namespace {
/* SBUS-RX object */
SbusRx sbus(&SBUS_UART);
/* Config */
SbusRxConfig config;
elapsedMillis time_ms;
static constexpr int32_t INIT_TIME_MS = 5000;
/* Interim data */
bool status, healthy, failsafe;
}

void SbusRxInit(const SbusRxConfig &cfg) {
  config = cfg;
  if (config.installed) {
    sbus.Begin();
    time_ms = 0;
    /* Try to confirm SBUS is working silently */
    while (time_ms < INIT_TIME_MS) {
      if (sbus.Read()) {
        return;
      }
    }
    /* Remind user to plug-in SBUS receiver */
    if (!sbus.Read()) {
      MsgInfo("\n\tWaiting for SBUS...");
      while (!sbus.Read()) {}
      MsgInfo("done.\n");
    }
  }
}

void SbusRxRead() {
  if (config.installed) {
    status = sbus.Read();
    if (status) {
      time_ms = 0;
    }
    healthy = (time_ms < HEALTHY_TIMEOUT_MS);
  }
}

void SbusRxInceptorData(InceptorData * const data) {
  data->installed = config.installed;
  if (config.installed) {
    data->new_data = status;
    if (data->new_data) {
      data->ch17 = sbus.ch17();
      data->ch18 = sbus.ch18();
      data->ch = sbus.ch();
      failsafe = sbus.failsafe();
    }
    if ((failsafe) || (!healthy)) {
      data->healthy = false;
    } else {
      data->healthy = true;
    }
  }
}
