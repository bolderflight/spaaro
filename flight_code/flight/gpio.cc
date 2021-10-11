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

#if defined(__FMU_R_V1__)

#include "flight/gpio.h"
#include "flight/global_defs.h"
#include "flight/config.h"
#include "flight/msg.h"
#include "polytools/polytools.h"

namespace {
/* GPIO config */
GpioConfig cfg_;
/* Digital output */
std::array<bool, NUM_GPIO_PINS> digout;
/* PWM effector */
std::array<bfs::PwmTx<1>, NUM_GPIO_PINS> pwm;
}  // namespace

void GpioInit(const GpioConfig &cfg) {
  MsgInfo("Intializing GPIO...");
  /* Copy the config */
  cfg_ = cfg;
  /* Check the pins and configure */
  for (std::size_t i = 0; i < NUM_GPIO_PINS; i++) {
    if (cfg_.channels[i].mode == GPIO_DIG_IN) {
      pinMode(GPIO_PINS[i], INPUT);
    } else if (cfg_.channels[i].mode == GPIO_DIG_OUT) {
      pinMode(GPIO_PINS[i], OUTPUT);
    } else if (cfg_.channels[i].mode == GPIO_PWM) {
      if (!pwm[i].Init(cfg_.channels[i].pwm)) {
        MsgError("Unable to initialize GPIO effectors.");
      } else {
        pwm[i].EnableServos();
      }
    }
  }
  MsgInfo("done.\n");
}
void GpioRead(GpioData * const data) {
  for (std::size_t i = 0; i < NUM_GPIO_PINS; i++) {
    if (cfg_.channels[i].mode == GPIO_AIN) {
      data->volt[i] = static_cast<float>(analogRead(GPIO_PINS[i])) *
                      AIN_VOLTAGE_SCALE;
      std::span<float> coef{cfg_.channels[i].analog.poly_coef,
            static_cast<std::size_t>(cfg_.channels[i].analog.num_coef)};
      data->val[i] = bfs::polyval<float>(coef, data->volt[i]);
    } else if (cfg_.channels[i].mode == GPIO_DIG_IN) {
      data->volt[i] = 0.0f;
      data->val[i] = static_cast<float>(digitalReadFast(GPIO_PINS[i]));
    } else {
      data->volt[i] = 0.0f;
      data->val[i] = 0.0f;
    }
  }
}
void GpioCmd(bool motor, bool servo, const ControlData &cmd) {
  for (std::size_t i = 0; i < NUM_GPIO_PINS; i++) {
    if (cfg_.channels[i].mode == GPIO_DIG_OUT) {
      digout[i] = (cmd.gpio[i] > 0) ? true : false;
    } else if (cfg_.channels[i].mode == GPIO_PWM) {
      if (motor) {
        pwm[i].EnableMotors();
      } else {
        pwm[i].DisableMotors();
      }
      if (servo) {
        pwm[i].EnableServos();
      } else {
        pwm[i].DisableServos();
      }
      float pwm_cmd[1] = {cmd.gpio[i]};
      pwm[i].Cmd(pwm_cmd);
    }
  }
}
void GpioWrite() {
  for (std::size_t i = 0; i < NUM_GPIO_PINS; i++) {
    if (cfg_.channels[i].mode == GPIO_DIG_OUT) {
      digitalWriteFast(GPIO_PINS[i], digout[i]);
    } else if (cfg_.channels[i].mode == GPIO_PWM) {
      pwm[i].Write();
    }
  }
}

#endif
