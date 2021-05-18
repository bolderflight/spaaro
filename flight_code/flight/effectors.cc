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

#include "flight/effectors.h"
#include "flight/global_defs.h"
#include "flight/config.h"
#include "flight/msg.h"

namespace {
  /* Effectors */
bfs::SbusTx<NUM_SBUS_CH> sbus;
bfs::PwmTx<NUM_PWM_PINS> pwm;
}  // namespace

void EffectorsInit(const EffectorConfig &cfg) {
  MsgInfo("Intializing effectors...");
  /* Init SBUS */
  if (!sbus.Init(cfg.sbus)) {
    MsgError("Unable to initialize SBUS effectors.");
  }
  /* Init PWM */
  if (!pwm.Init(cfg.pwm)) {
    MsgError("Unable to initialize PWM effectors.");
  }
  /* Servos enabled by default */
  sbus.EnableServos();
  pwm.EnableServos();
  MsgInfo("done.\n");
}
void EffectorsCmd(bool motor, bool servo, const ControlData &cmd) {
  /* Enable / disable motors */
  if (motor) {
    sbus.EnableMotors();
    pwm.EnableMotors();
  } else {
    sbus.DisableMotors();
    pwm.DisableMotors();
  }
  /* Enable / disable servos */
  if (servo) {
    sbus.EnableServos();
    pwm.EnableServos();
  } else {
    sbus.DisableServos();
    pwm.DisableServos();
  }
  /* Set effector commands */
  sbus.Cmd(cmd.sbus);
  pwm.Cmd(cmd.pwm);
}
void EffectorsWrite() {
  /* Write the effector commands */
  sbus.Write();
  pwm.Write();
}
