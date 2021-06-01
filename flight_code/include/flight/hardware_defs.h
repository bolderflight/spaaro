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

#ifndef FLIGHT_CODE_INCLUDE_FLIGHT_HARDWARE_DEFS_H_
#define FLIGHT_CODE_INCLUDE_FLIGHT_HARDWARE_DEFS_H_

#include <array>
#include <cstdint>
#include <cmath>
#include "core/core.h"
#include "imu/imu.h"

inline constexpr int8_t NUM_AIN_PINS = 8;

#ifdef __FMU_R_V2_BETA__

/* Messages */
inline constexpr usb_serial_class &MSG_BUS = Serial;
/* Frame rate */
inline constexpr bfs::FrameRate FRAME_RATE_HZ = bfs::FRAME_RATE_50HZ;
inline constexpr int16_t FRAME_PERIOD_MS = 1000 /
                                           static_cast<uint8_t>(FRAME_RATE_HZ);
/* Inceptor / Effector */
inline constexpr int8_t NUM_SBUS_CH = 16;
inline constexpr int8_t NUM_PWM_PINS = 8;
inline constexpr HardwareSerial &SBUS_UART = Serial2;
inline constexpr std::array<int8_t, NUM_PWM_PINS> PWM_PINS = {37, 22, 23, 6,
                                                              5, 4, 3, 2};
/* 90% of the frame period */
inline constexpr float EFFECTOR_DELAY_US = FRAME_PERIOD_MS * 0.9f * 1e3;
/* IMU */
inline constexpr SPIClass &IMU_SPI_BUS = SPI;
inline constexpr int8_t IMU_CS = 36;
inline constexpr int8_t IMU_DRDY = 35;
inline constexpr int8_t VN_CS = 34;
inline constexpr int8_t VN_DRDY = 33;
/* Pressure transducers */
inline constexpr TwoWire &PRES_I2C_BUS = Wire;
inline constexpr SPIClass &PRES_SPI_BUS = SPI;
inline constexpr int8_t PRES_CS = 32;
/* Voltage */
inline constexpr int ANALOG_RESOLUTION_BITS = 12;
inline constexpr float VOLTAGE_RANGE = 3.3;
inline constexpr float ANALOG_COUNT_RANGE =
  std::pow(2.0f, ANALOG_RESOLUTION_BITS) - 1.0f;
inline constexpr std::array<int8_t, NUM_AIN_PINS> AIN_PINS = {38, 39, 40, 41,
                                                              24, 25, 26, 27};
inline constexpr float AIN_VOLTAGE_SCALE = VOLTAGE_RANGE / ANALOG_COUNT_RANGE;
#else

/* Messages */
inline constexpr usb_serial_class &MSG_BUS = Serial;
/* Frame rate */
inline constexpr bfs::FrameRate FRAME_RATE_HZ = bfs::FRAME_RATE_50HZ;
inline constexpr int16_t FRAME_PERIOD_MS = 1000 /
                                           static_cast<uint8_t>(FRAME_RATE_HZ);
/* Inceptor / Effector */
inline constexpr int8_t NUM_SBUS_CH = 16;
inline constexpr int8_t NUM_PWM_PINS = 8;
inline constexpr HardwareSerial &SBUS_UART = Serial2;
inline constexpr std::array<int8_t, NUM_PWM_PINS> PWM_PINS = {21, 22, 23, 2,
                                                              3, 4, 5, 6};
/* 90% of the frame period */
inline constexpr float EFFECTOR_DELAY_US = FRAME_PERIOD_MS * 0.9f * 1e3;
/* IMU */
inline constexpr SPIClass &IMU_SPI_BUS = SPI;
inline constexpr int8_t IMU_CS = 24;
inline constexpr int8_t IMU_DRDY = 27;
inline constexpr int8_t VN_CS = 25;
inline constexpr int8_t VN_DRDY = 28;
/* Pressure transducers */
inline constexpr TwoWire &PRES_I2C_BUS = Wire1;
inline constexpr SPIClass &PRES_SPI_BUS = SPI;
inline constexpr int8_t PRES_CS = 26;
/* Voltage */
inline constexpr int ANALOG_RESOLUTION_BITS = 16;
inline constexpr float VOLTAGE_RANGE = 3.3;
inline constexpr float ANALOG_COUNT_RANGE =
  std::pow(2.0f, ANALOG_RESOLUTION_BITS) - 1.0f;
inline constexpr unsigned int INPUT_VOLTAGE_PIN = 15;
inline constexpr float INPUT_VOLTAGE_SCALE = VOLTAGE_RANGE /
  ANALOG_COUNT_RANGE * (10000.0f + 1000.0f) / 1000.0f;
inline constexpr unsigned int REGULATED_VOLTAGE_PIN = A22;
inline constexpr float REGULATED_VOLTAGE_SCALE = VOLTAGE_RANGE /
  ANALOG_COUNT_RANGE * (1000.0f + 1000.0f) / 1000.0f;
inline constexpr unsigned int SBUS_VOLTAGE_PIN = A21;
inline constexpr float SBUS_VOLTAGE_SCALE = VOLTAGE_RANGE /
  ANALOG_COUNT_RANGE * (1000.0f + 499.0f) / 499.0f;
inline constexpr unsigned int PWM_VOLTAGE_PIN = 39;
inline constexpr float PWM_VOLTAGE_SCALE = VOLTAGE_RANGE /
  ANALOG_COUNT_RANGE * (1000.0f + 499.0f) / 499.0f;

#endif

#endif  // FLIGHT_CODE_INCLUDE_FLIGHT_HARDWARE_DEFS_H_
