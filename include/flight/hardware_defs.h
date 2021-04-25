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

#include <cmath>
#include <array>
#include "core/core.h"

/* Messages */
static constexpr usb_serial_class &MSG_BUS = Serial;
/* Frame rate */
static constexpr int FRAME_RATE_HZ = 50;
static constexpr float FRAME_PERIOD_S = 1.0f / FRAME_RATE_HZ;
/* Inceptor / Effector */
static constexpr HardwareSerial &SBUS_UART = Serial2;
static constexpr int NUM_SBUS_CH = 16;
static constexpr int NUM_PWM_PINS = 8;
static constexpr std::array<int, NUM_PWM_PINS> PWM_PINS = {21, 22, 23, 2, 3,
                                                           4, 5, 6};
/* 90% of the frame period */
static constexpr float EFFECTOR_DELAY_US = FRAME_PERIOD_S * 0.9f * 1e6;
/* IMU */
static constexpr SPIClass &IMU_SPI_BUS = SPI;
static constexpr unsigned int IMU_CS = 24;
static constexpr unsigned int IMU_DRDY = 27;
// /* Pitot Static */
// #ifdef HAVE_PITOT_STATIC
// static constexpr TwoWire &STATIC_PRES_I2C_BUS = Wire1;
// static constexpr bfs::Ams5915::Transducer STATIC_PRES_TRANSDUCER =
//   bfs::Ams5915::AMS5915_1200_B;
// static constexpr unsigned int STATIC_PRES_ADDR = 0x10;
// static constexpr TwoWire &DIFF_PRES_I2C_BUS = Wire1;
// static constexpr unsigned int DIFF_PRES_ADDR = 0x11;
// #endif
// static constexpr SPIClass &FMU_PRES_SPI_BUS = SPI;
// static constexpr unsigned int FMU_PRES_CS = 26;
// /* Voltage */
// static constexpr int ANALOG_RESOLUTION_BITS = 16;
// static constexpr float VOLTAGE_RANGE = 3.3;
// static constexpr float ANALOG_COUNT_RANGE =
//   std::pow(2.0f, ANALOG_RESOLUTION_BITS) - 1.0f;
// static constexpr unsigned int INPUT_VOLTAGE_PIN = 15;
// static constexpr float INPUT_VOLTAGE_SCALE = VOLTAGE_RANGE /
//   ANALOG_COUNT_RANGE * (10000.0f + 1000.0f) / 1000.0f;
// static constexpr unsigned int REGULATED_VOLTAGE_PIN = A22;
// static constexpr float REGULATED_VOLTAGE_SCALE = VOLTAGE_RANGE /
//   ANALOG_COUNT_RANGE * (1000.0f + 1000.0f) / 1000.0f;
// static constexpr unsigned int SBUS_VOLTAGE_PIN = A21;
// static constexpr float SBUS_VOLTAGE_SCALE = VOLTAGE_RANGE /
//   ANALOG_COUNT_RANGE * (1000.0f + 499.0f) / 499.0f;
// static constexpr unsigned int PWM_VOLTAGE_PIN = 39;
// static constexpr float PWM_VOLTAGE_SCALE = VOLTAGE_RANGE /
//   ANALOG_COUNT_RANGE * (1000.0f + 499.0f) / 499.0f;
// /* Telemetry */
// static constexpr HardwareSerial &TELEM_UART = Serial4;
// static constexpr unsigned int TELEM_BAUD = 57600;

#endif  // INCLUDE_FLIGHT_HARDWARE_DEFS_H_
