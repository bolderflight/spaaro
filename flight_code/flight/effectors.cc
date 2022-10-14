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

#include "flight/effectors.h"
#include "global_defs.h"
#include "flight/config.h"
#include "flight/msg.h"

namespace {
/* Effectors */
// bfs::SbusTx sbus;
// bfs::PwmTx<NUM_PWM_PINS> pwm;
}  // namespace

void EffectorsInit() {
  // MsgInfo("Intializing effectors...");
  // /* Init SBUS */
  // sbus.Init(&SBUS_UART);
  // /* Init PWM */
  // pwm.Init(PWM_PINS);
  // MsgInfo("done.\n");
}
void EffectorsCmd(const VmsData &vms) {
  // /* Set effector commands */
  // sbus.ch(vms.sbus.cnt);
  // sbus.ch17(vms.sbus.ch17);
  // sbus.ch18(vms.sbus.ch18);
  // pwm.ch(vms.pwm.cnt);
}
void EffectorsWrite() {
  // /* Write the effector commands */
  // sbus.Write();
  // pwm.Write();
}
