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

#if defined(__FMU_R_MINI_V1__) || defined(__FMU_R_V2__)

#include "drivers/power-module.h"
#include "global_defs.h"
#include "flight/config.h"
#include "flight/msg.h"

namespace {
PowerModuleConfig cfg_;
}  // namespace

void PowerModuleInit(const PowerModuleConfig &cfg) {
  cfg_ = cfg;
}

void PowerModuleRead(PowerModuleData * const data) {
  data->voltage_v = static_cast<float>(analogRead(PWR_MOD_VOLTAGE_PIN)) *
                    AIN_VOLTAGE_SCALE * cfg_.volts_per_volt;
  data->current_ma = static_cast<float>(analogRead(PWR_MOD_CURRENT_PIN)) *
                     AIN_VOLTAGE_SCALE * cfg_.amps_per_volt * 1000.0f;
}

#endif
