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

#ifndef FLIGHT_CODE_INCLUDE_HARDWARE_DEFS_H_
#define FLIGHT_CODE_INCLUDE_HARDWARE_DEFS_H_

#include <array>
#include <cstdint>
#include <cmath>
#include "core/core.h"

/* FMU-R mini */
#if defined(__FMU_R_MINI_V1__)
/* Messages */
inline constexpr usb_serial_class &MSG_BUS = Serial;
/* Inceptor / Effector */
inline constexpr int8_t NUM_SBUS_CH = 16;
inline constexpr int8_t NUM_PWM_PINS = 8;
inline constexpr HardwareSerial &INCEPT_UART = Serial1;
inline constexpr int8_t PWM_PINS[NUM_PWM_PINS] = {6, 9, 44, 3,
                                                  33, 5, 2, 4};
/* UART */
inline constexpr HardwareSerial &GNSS1_UART = Serial3;
inline constexpr HardwareSerial &GNSS2_UART = Serial2;
inline constexpr HardwareSerial &AUX_UART   = Serial4;
inline constexpr HardwareSerial &TELEM_UART = Serial7;
/* Frame period */
inline constexpr float MAG_RATE_HZ = 100.0f;
inline constexpr float FMU_MAG_PERIOD_MS = 10.0f;
inline constexpr float FRAME_RATE_HZ = 100.0f;
inline constexpr float FRAME_PERIOD_MS = 10.0f;
/* 90% of the frame period */
inline constexpr float EFFECTOR_DELAY_US = FRAME_PERIOD_MS * 0.9f * 1e3;
/* SPI BUS */
inline constexpr SPIClass &SPI_BUS = SPI;
/* I2C Bus */
inline constexpr TwoWire &I2C_BUS = Wire;
/* IMU */
inline constexpr int8_t IMU_SRD = 9;
inline constexpr int8_t IMU_CS = 10;
inline constexpr int8_t IMU_DRDY = 43;
/* PRES */
inline constexpr int8_t PRES_CS = 45;
/* Mag */
inline constexpr int8_t MAG_CS = 42;
/* Analog */
inline constexpr int8_t NUM_AIN_PINS = 6;
inline constexpr int ANALOG_RESOLUTION_BITS = 12;
inline constexpr float VOLTAGE_RANGE = 3.3;
inline constexpr float ANALOG_COUNT_RANGE =
  std::pow(2.0f, ANALOG_RESOLUTION_BITS) - 1.0f;
inline constexpr int8_t AIN_PINS[NUM_AIN_PINS] = {24, 25, 20, 26, 21, 27};
inline constexpr float AIN_VOLTAGE_SCALE = VOLTAGE_RANGE / ANALOG_COUNT_RANGE;
// /* Voltage and current */
inline constexpr int8_t PWR_MOD_VOLTAGE_PIN = 23;
inline constexpr int8_t PWR_MOD_CURRENT_PIN = 22;

/* FMU-R V2 */
#elif defined(__FMU_R_V2__)
/* Messages */
inline constexpr usb_serial_class &MSG_BUS = Serial;
/* Inceptor / Effector */
inline constexpr int8_t NUM_SBUS_CH = 16;
inline constexpr int8_t NUM_PWM_PINS = 8;
inline constexpr HardwareSerial &INCEPT_UART = Serial2;
inline constexpr int8_t PWM_PINS[NUM_PWM_PINS] = {37, 9, 10, 6,
                                                  5, 4, 3, 2};
/* UART */
inline constexpr HardwareSerial &GNSS1_UART = Serial3;
inline constexpr HardwareSerial &GNSS2_UART = Serial4;
inline constexpr HardwareSerial &AUX_UART   = Serial5;
inline constexpr HardwareSerial &TELEM_UART = Serial7;
/* Frame period */
inline constexpr float MAG_RATE_HZ = 100.0f;
inline constexpr float FMU_MAG_PERIOD_MS = 10.0f;
inline constexpr float FRAME_RATE_HZ = 100.0f;
inline constexpr float FRAME_PERIOD_MS = 10.0f;
/* 90% of the frame period */
inline constexpr float EFFECTOR_DELAY_US = FRAME_PERIOD_MS * 0.9f * 1e3;
/* SPI BUS */
inline constexpr SPIClass &SPI_BUS = SPI;
/* I2C Bus */
inline constexpr TwoWire &I2C_BUS = Wire;
/* IMU */
inline constexpr int8_t IMU_CS = 36;
inline constexpr int8_t IMU_DRDY = 35;
/* PRES */
inline constexpr int8_t PRES_CS = 32;
/* VectorNav */
inline constexpr int8_t VN_CS = 34;
inline constexpr int8_t VN_DRDY = 33;
/* Analog */
inline constexpr int8_t NUM_AIN_PINS = 8;
inline constexpr int ANALOG_RESOLUTION_BITS = 12;
inline constexpr float VOLTAGE_RANGE = 3.3;
inline constexpr float ANALOG_COUNT_RANGE =
  std::pow(2.0f, ANALOG_RESOLUTION_BITS) - 1.0f;
inline constexpr int8_t AIN_PINS[NUM_AIN_PINS] = {38, 39, 41, 40,
                                                  24, 25, 26, 27};
inline constexpr float AIN_VOLTAGE_SCALE = VOLTAGE_RANGE / ANALOG_COUNT_RANGE;
/* Voltage and current */
inline constexpr int8_t PWR_MOD_VOLTAGE_PIN = 23;
inline constexpr int8_t PWR_MOD_CURRENT_PIN = 22;

/* FMU-R V2-Beta */
#elif defined(__FMU_R_V2_BETA__)
/* Messages */
inline constexpr usb_serial_class &MSG_BUS = Serial;
/* Inceptor / Effector */
inline constexpr int8_t NUM_SBUS_CH = 16;
inline constexpr int8_t NUM_PWM_PINS = 8;
inline constexpr HardwareSerial &INCEPT_UART = Serial2;
inline constexpr int8_t PWM_PINS[NUM_PWM_PINS] = {37, 22, 23, 6,
                                                  5, 4, 3, 2};
/* UART */
inline constexpr HardwareSerial &GNSS1_UART = Serial3;
inline constexpr HardwareSerial &GNSS2_UART = Serial4;
inline constexpr HardwareSerial &AUX_UART   = Serial5;
inline constexpr HardwareSerial &TELEM_UART = Serial7;
/* Frame period */
inline constexpr float MAG_RATE_HZ = 100.0f;
inline constexpr float FMU_MAG_PERIOD_MS = 10.0f;
inline constexpr float FRAME_RATE_HZ = 100.0f;
inline constexpr float FRAME_PERIOD_MS = 10.0f;
/* 90% of the frame period */
inline constexpr float EFFECTOR_DELAY_US = FRAME_PERIOD_MS * 0.9f * 1e3;
/* SPI BUS */
inline constexpr SPIClass &SPI_BUS = SPI;
/* I2C Bus */
inline constexpr TwoWire &I2C_BUS = Wire;
/* IMU */
inline constexpr int8_t IMU_CS = 36;
inline constexpr int8_t IMU_DRDY = 35;
/* PRES */
inline constexpr int8_t PRES_CS = 32;
/* VectorNav */
inline constexpr int8_t VN_CS = 34;
inline constexpr int8_t VN_DRDY = 33;
/* Analog */
inline constexpr int8_t NUM_AIN_PINS = 8;
inline constexpr int ANALOG_RESOLUTION_BITS = 12;
inline constexpr float VOLTAGE_RANGE = 3.3;
inline constexpr float ANALOG_COUNT_RANGE =
  std::pow(2.0f, ANALOG_RESOLUTION_BITS) - 1.0f;
inline constexpr int8_t AIN_PINS[NUM_AIN_PINS] = {38, 39, 41, 40,
                                                  24, 25, 26, 27};
inline constexpr float AIN_VOLTAGE_SCALE = VOLTAGE_RANGE / ANALOG_COUNT_RANGE;

/* FMU-R V1 */
#else
/* Messages */
inline constexpr usb_serial_class &MSG_BUS = Serial;
/* Inceptor / Effector */
inline constexpr int8_t NUM_SBUS_CH = 16;
inline constexpr int8_t NUM_PWM_PINS = 8;
inline constexpr HardwareSerial &INCEPT_UART = Serial2;
inline constexpr int8_t PWM_PINS[NUM_PWM_PINS] = {21, 22, 23, 2,
                                                  3, 4, 5, 6};
/* UART */
inline constexpr HardwareSerial &GNSS_UART = Serial3;
inline constexpr HardwareSerial &TELEM_UART = Serial4;
/* Frame period */
inline constexpr float MAG_RATE_HZ = 8.0f;
inline constexpr float FMU_MAG_PERIOD_MS = 125.0f;
inline constexpr float FRAME_RATE_HZ = 50.0f;
inline constexpr float FRAME_PERIOD_MS = 20.0f;
/* 90% of the frame period */
inline constexpr float EFFECTOR_DELAY_US = FRAME_PERIOD_MS * 0.9f * 1e3;
/* SPI BUS */
inline constexpr SPIClass &SPI_BUS = SPI;
/* I2C Bus */
inline constexpr TwoWire &I2C_BUS = Wire1;
/* IMU */
inline constexpr int8_t IMU_CS = 24;
inline constexpr int8_t IMU_DRDY = 27;
/* PRES */
inline constexpr int8_t PRES_CS = 26;
/* VectorNav */
inline constexpr int8_t VN_CS = 25;
inline constexpr int8_t VN_DRDY = 28;
/* Analog */
inline constexpr int8_t NUM_AIN_PINS = 2;
inline constexpr int ANALOG_RESOLUTION_BITS = 16;
inline constexpr float VOLTAGE_RANGE = 3.3;
inline constexpr float ANALOG_COUNT_RANGE =
  std::pow(2.0f, ANALOG_RESOLUTION_BITS) - 1.0f;
inline constexpr int8_t AIN_PINS[NUM_AIN_PINS] = {14, 16};
inline constexpr float AIN_VOLTAGE_SCALE = VOLTAGE_RANGE / ANALOG_COUNT_RANGE;
inline constexpr int8_t INPUT_VOLTAGE_PIN = 15;
inline constexpr float INPUT_VOLTAGE_SCALE = VOLTAGE_RANGE /
  ANALOG_COUNT_RANGE * (10000.0f + 1000.0f) / 1000.0f;
inline constexpr int8_t REGULATED_VOLTAGE_PIN = A22;
inline constexpr float REGULATED_VOLTAGE_SCALE = VOLTAGE_RANGE /
  ANALOG_COUNT_RANGE * (1000.0f + 1000.0f) / 1000.0f;
inline constexpr int8_t SBUS_VOLTAGE_PIN = A21;
inline constexpr float SBUS_VOLTAGE_SCALE = VOLTAGE_RANGE /
  ANALOG_COUNT_RANGE * (1000.0f + 499.0f) / 499.0f;
inline constexpr int8_t PWM_VOLTAGE_PIN = 39;
inline constexpr float PWM_VOLTAGE_SCALE = VOLTAGE_RANGE /
  ANALOG_COUNT_RANGE * (1000.0f + 499.0f) / 499.0f;
#endif

#endif  // FLIGHT_CODE_INCLUDE_HARDWARE_DEFS_H_
