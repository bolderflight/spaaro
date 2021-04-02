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
#include "flight/effector.h"
#include "effector/effector.h"
#include "sbus/sbus.h"
#include "pwm/pwm.h"
#include "polytools/polytools.h"

namespace {
bfs::SbusTx sbus_(&SBUS_UART);
bfs::PwmTx<NUM_PWM_PINS> pwm_(PWM_PINS);
/* Commands */
std::array<uint16_t, 16> sbus_cmds_ = {0};
std::array<uint16_t, NUM_PWM_PINS> pwm_cmds_ = {0};
/* Enable / disable servos and motors */
bool servo_enable_ = true;
bool motor_enable_ = false;
}  // namespace

void EffectorInit() {
  MsgInfo("Initializing effectors...");
  sbus_.Begin();
  pwm_.Begin();
  MsgInfo("done.\n");
}
void EffectorCmd(const EffectorCmds &ref) {
  for (const auto& eff : sbus_effectors) {
    sbus_cmds_[eff.ch] =
      eff.Cmd(ref.sbus[eff.ch], servo_enable_, motor_enable_);
  }
  for (const auto& eff : pwm_effectors) {
    pwm_cmds_[eff.ch] =
      eff.Cmd(ref.pwm[eff.ch], servo_enable_, motor_enable_);
  }
  sbus_.tx_channels(sbus_cmds_);
  pwm_.tx_channels(pwm_cmds_);
}
void EffectorWrite() {
  sbus_.Write();
  pwm_.Write();
}
