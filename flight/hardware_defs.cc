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
#include <array>
#include <cstdint>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "core/core.h"

/* Messages */
usb_serial_class &MSG_BUS = Serial;
/* Frame rate */
int32_t FRAME_RATE_HZ = 50;
int32_t FRAME_PERIOD_MS = 1000 / FRAME_RATE_HZ;
/* Inceptor / Effector */
static constexpr int8_t NUM_SBUS_CH = 16;
static constexpr int8_t NUM_PWM_PINS = 8;
HardwareSerial &SBUS_UART = Serial2;
int8_t PWM_PINS[NUM_PWM_PINS] = {21, 22, 23, 2, 3, 4, 5, 6};
/* 90% of the frame period */
float EFFECTOR_DELAY_US = FRAME_PERIOD_MS * 0.9f * 1e3;
/* IMU */
SPIClass &IMU_SPI_BUS = SPI;
int8_t IMU_CS = 24;
int8_t IMU_DRDY = 27;
Eigen::Matrix3f IMU_ROTATION = Eigen::Matrix3f::Identity();
