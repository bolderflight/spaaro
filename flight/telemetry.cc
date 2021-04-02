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
#include "flight/telemetry.h"
#include "mavlink_types.h"
#include "common/mavlink.h"

namespace {
/* MAV Link message data */
mavlink_message_t msg_;
mavlink_status_t status_;
uint16_t msg_len_;
uint8_t msg_buf_[MAVLINK_MAX_PACKET_LEN];
/* Timers */
elapsedMillis heartbeat_timer_ms_;
static constexpr int HEARTBEAT_PERIOD_MS_ = 1000;

}  // namespace

void TelemInit() {
  MsgInfo("Initializing telemetry...");
  TELEM_UART.begin(TELEM_BAUD);
  MsgInfo("done.\n");
}
void TelemTxRx(const AircraftData &ref) {


  if (heartbeat_timer_ms_ > HEARTBEAT_PERIOD_MS_) {

  }
  
}
