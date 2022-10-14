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

#include "flight/sys.h"
#include "hardware_defs.h"
#include "global_defs.h"
#include "flight/msg.h"

namespace {
/* Frame time */
int64_t frame_start_us_;
int32_t frame_time_us_ = 0;
}  // namespace

void SysInit() {
  MsgInfo("Initializing system...");
  /* Pullup CS pins */
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_V1__)
  pinMode(IMU_CS, OUTPUT);
  pinMode(VN_CS, OUTPUT);
  pinMode(PRES_CS, OUTPUT);
  digitalWriteFast(IMU_CS, HIGH);
  digitalWriteFast(VN_CS, HIGH);
  digitalWriteFast(PRES_CS, HIGH);
  #elif defined(__FMU_R_MINI_V1__)
  pinMode(IMU_CS, OUTPUT);
  pinMode(MAG_CS, OUTPUT);
  pinMode(PRES_CS, OUTPUT);
  digitalWriteFast(IMU_CS, HIGH);
  digitalWriteFast(MAG_CS, HIGH);
  digitalWriteFast(PRES_CS, HIGH);
  #endif
  /* Initialize buses */
  #if defined(__FMU_R_MINI_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  /* I2C */
  Wire.begin();
  Wire.setClock(400000);
  #endif
  #if defined(__FMU_R_V1__)
  /* I2C */
  Wire1.begin();
  Wire1.setClock(400000);
  #endif
  SPI.begin();
  /* Setup analog for voltage monitoring */
  analogReadResolution(ANALOG_RESOLUTION_BITS);
  MsgInfo("done.\n");
}

void SysRead(SysData * const ptr) {
  if (!ptr) {return;}
  frame_start_us_ = micros64();
  ptr->sys_time_us = frame_start_us_;
  ptr->frame_time_us = frame_time_us_;
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
