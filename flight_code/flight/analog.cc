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

#include "flight/analog.h"
#include "flight/global_defs.h"
#include "flight/config.h"
#include "flight/msg.h"
#include "polytools/polytools.h"

namespace {
/* Analog config */
AnalogConfig cfg_;
}  // namespace

void AnalogInit(const AnalogConfig &cfg) {
  /* Copy the config */
  cfg_ = cfg;
}
void AnalogRead(AnalogData * const data) {
#ifdef __FMU_R_V2_BETA__
  for (std::size_t i = 0; i < NUM_AIN_PINS; i++) {
    data->volt[i] = static_cast<float>(analogRead(AIN_PINS[i])) *
                             AIN_VOLTAGE_SCALE;
    std::span<float> coef{cfg_.channels[i].poly_coef,
          static_cast<std::size_t>(cfg_.channels[i].num_coef)};
    data->val[i] = bfs::polyval<float>(coef, data->volt[i]);
  }
#endif
}