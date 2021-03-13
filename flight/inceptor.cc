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

#include "flight/global_defs.h"
#include "flight/hardware_defs.h"
#include "flight/config.h"
#include "flight/msg.h"
#include "flight/inceptor.h"
#include "sbus/sbus.h"
#include "polytools/polytools.h"

namespace {
/* SBUS object */
bfs::SbusRx sbus_(&SBUS_UART);
/* SBUS channel mapping */
static constexpr int THR_CH_ = 0;
static constexpr int ROLL_CH_ = 1;
static constexpr int PITCH_CH_ = 2;
static constexpr int YAW_CH_ = 3;
static constexpr int MODE0_CH_ = 4;
static constexpr int MODE1_CH_ = 5;
static constexpr int THR_EN_CH_ = 6;
/* Polyval coefficients for pitch, roll, yaw command, map SBUS to -1 to +1 */
static constexpr std::array<float, 2> ATT_POLY_COEF_ =
  {0.001220256253813301f, -1.209884075655888f};
/* Polyval coefficients for throttle, map SBUS to 0 to +1 */
static constexpr std::array<float, 2> THR_POLY_COEF_ =
  {0.0006101281269066503f, -0.1049420378279439f};
/* Polyval coefficients for switches, map SBUS to 0 to +1 */
static constexpr std::array<float, 2> MODE_COEF_ =
  {0.0006101281269066503f, -0.1049420378279439f};
/* Mode0, Mode1, throttle enabled intermediate values */
float mode0_, mode1_, thr_en_;
}  // namespace

void InceptorInit() {
  MsgInfo("Initializing inceptors...");
  while (!sbus_.Begin()) {}
  MsgInfo("done.\n");
}
void InceptorRead(InceptorData * const ptr) {
  if (!ptr) {return;}
  ptr->new_data = sbus_.Read();
  if (ptr->new_data) {
    /* Throttle and attitude commands */
    ptr->throttle = bfs::polyval(THR_POLY_COEF_,
                    static_cast<float>(sbus_.rx_channels()[THR_CH_]));
    ptr->roll = bfs::polyval(ATT_POLY_COEF_,
                static_cast<float>(sbus_.rx_channels()[ROLL_CH_]));
    ptr->pitch = bfs::polyval(ATT_POLY_COEF_,
                 static_cast<float>(sbus_.rx_channels()[PITCH_CH_]));
    ptr->yaw = bfs::polyval(ATT_POLY_COEF_,
               static_cast<float>(sbus_.rx_channels()[YAW_CH_]));
    /* Mode 0, 3 position switch */
    mode0_ = bfs::polyval(MODE_COEF_,
             static_cast<float>(sbus_.rx_channels()[MODE0_CH_]));
    if (mode0_ < 0.3f) {
      ptr->mode0 = 2;
    } else if (mode0_ < 0.6f) {
      ptr->mode0 = 1;
    } else {
      ptr->mode0 = 0;
    }
    /* Mode 1, 3 position switch */
    mode1_ = bfs::polyval(MODE_COEF_,
             static_cast<float>(sbus_.rx_channels()[MODE1_CH_]));
    if (mode1_ < 0.3f) {
      ptr->mode1 = 2;
    } else if (mode1_ < 0.6f) {
      ptr->mode1 = 1;
    } else {
      ptr->mode1 = 0;
    }
    /* Throttle enabled, on/off */
    thr_en_ = bfs::polyval(MODE_COEF_,
              static_cast<float>(sbus_.rx_channels()[THR_EN_CH_]));
    ptr->throttle_en = thr_en_ > 0.5f;
    /* Lost frame and failsafe */
    ptr->lost_frame = sbus_.lost_frame();
    ptr->failsafe = sbus_.failsafe();
  }
}
