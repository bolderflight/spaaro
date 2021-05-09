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
#include "imu/imu.h"

/* Messages */
extern usb_serial_class &MSG_BUS;
/* Frame rate */
inline constexpr bfs::FrameRate FRAME_RATE_HZ = bfs::FRAME_RATE_50HZ;
inline constexpr int16_t FRAME_PERIOD_MS = 1000 /
                                           static_cast<uint8_t>(FRAME_RATE_HZ);
/* Inceptor / Effector */
inline constexpr int8_t NUM_SBUS_CH = 16;
inline constexpr int8_t NUM_PWM_PINS = 8;
extern HardwareSerial &SBUS_UART;
inline constexpr std::array<int8_t, NUM_PWM_PINS> PWM_PINS = {21, 22, 23, 2,
                                                              3, 4, 5, 6};
/* 90% of the frame period */
extern float EFFECTOR_DELAY_US;
/* IMU */
extern SPIClass &IMU_SPI_BUS;
extern int8_t IMU_CS;
extern int8_t IMU_DRDY;
extern Eigen::Matrix3f IMU_ROTATION;
extern int8_t VN_CS;
extern int8_t VN_DRDY;
/* Pressure transducers */
extern TwoWire &PRES_I2C_BUS;
extern SPIClass &PRES_SPI_BUS;
extern int8_t PRES_CS;
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



#endif  // INCLUDE_FLIGHT_HARDWARE_DEFS_H_
