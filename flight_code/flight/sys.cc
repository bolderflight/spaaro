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

#include "flight/sys.h"
#include "flight/hardware_defs.h"
#include "flight/global_defs.h"

namespace {
/* Frame time */
int64_t frame_start_us_;
int32_t frame_time_us_ = 0;
}  // namespace

void SysInit() {
  /* Pullup CS pins */
  pinMode(IMU_CS, OUTPUT);
  pinMode(VN_CS, OUTPUT);
  pinMode(PRES_CS, OUTPUT);
  digitalWriteFast(IMU_CS, HIGH);
  digitalWriteFast(VN_CS, HIGH);
  digitalWriteFast(PRES_CS, HIGH);
  /* Initialize buses */
  Wire.begin();
  Wire.setClock(400000);
  Wire1.begin();
  Wire1.setClock(400000);
  SPI.begin();
  /* Setup analog for voltage monitoring */
  analogReadResolution(ANALOG_RESOLUTION_BITS);
}
void SysRead(SysData * const ptr) {
  if (!ptr) {return;}
  frame_start_us_ = micros64();
  ptr->sys_time_us = frame_start_us_;
  ptr->sys_time_s = static_cast<double>(ptr->sys_time_us) / 1e6;
  ptr->frame_time_us = frame_time_us_;
  ptr->frame_time_s = static_cast<float>(ptr->frame_time_us) / 1000000.0f;
  #if defined(__FMU_R_V1__)
  ptr->input_volt = static_cast<float>(analogRead(INPUT_VOLTAGE_PIN)) *
                    INPUT_VOLTAGE_SCALE;
  ptr->reg_volt = static_cast<float>(analogRead(REGULATED_VOLTAGE_PIN)) *
                  REGULATED_VOLTAGE_SCALE;
  ptr->sbus_volt = static_cast<float>(analogRead(SBUS_VOLTAGE_PIN)) *
                   SBUS_VOLTAGE_SCALE;
  ptr->pwm_volt = static_cast<float>(analogRead(PWM_VOLTAGE_PIN)) *
                  PWM_VOLTAGE_SCALE;
  #endif
}
void SysFrameEnd() {
  frame_time_us_ = static_cast<int32_t>(micros64() - frame_start_us_);
}
