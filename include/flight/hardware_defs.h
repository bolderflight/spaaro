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

#ifndef INCLUDE_FLIGHT_HARDWARE_DEFS_H_
#define INCLUDE_FLIGHT_HARDWARE_DEFS_H_

#include <array>
#include <cstdint>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "core/core.h"

/* Messages */
extern usb_serial_class &MSG_BUS;
/* Frame rate */
extern int32_t FRAME_RATE_HZ;
extern int32_t FRAME_PERIOD_MS;
/* Inceptor / Effector */
extern HardwareSerial &SBUS_UART;
extern int8_t PWM_PINS[];
/* 90% of the frame period */
extern float EFFECTOR_DELAY_US;
/* IMU */
extern SPIClass &IMU_SPI_BUS;
extern int8_t IMU_CS;
extern int8_t IMU_DRDY;
extern Eigen::Matrix3f IMU_ROTATION;
/* Airdata */
extern TwoWire &PRES_I2C_BUS;
extern SPIClass &PRES_SPI_BUS;


#endif  // INCLUDE_FLIGHT_HARDWARE_DEFS_H_
