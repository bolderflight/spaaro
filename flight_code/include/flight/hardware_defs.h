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

#ifndef FLIGHT_CODE_INCLUDE_FLIGHT_HARDWARE_DEFS_H_
#define FLIGHT_CODE_INCLUDE_FLIGHT_HARDWARE_DEFS_H_

#include <array>
#include "core/core.h"

/* FMU-R V2 */
#if defined(__FMU_R_V2__)
/* Messages */
inline constexpr usb_serial_class &MSG_BUS = Serial;
/* Inceptor / Effector */
inline constexpr int32_t SBUS_HEALTHY_TIMEOUT_MS = 100;
inline constexpr int8_t NUM_SBUS_CH = 16;
inline constexpr int8_t NUM_PWM_PINS = 8;
inline constexpr HardwareSerial &SBUS_UART = Serial2;
inline constexpr std::array<int8_t, NUM_PWM_PINS> PWM_PINS = {37, 9, 10, 6,
                                                               5, 4, 3, 2};
/* Frame period */
inline constexpr float FRAME_RATE_HZ = 100.0f;
inline constexpr float FRAME_PERIOD_MS = 10.0f;
/* 90% of the frame period */
inline constexpr float EFFECTOR_DELAY_US = FRAME_PERIOD_MS * 0.9f * 1e3;
/* SPI BUS */
inline constexpr SPIClass &SPI_BUS = SPI;
/* I2C Bus */
inline constexpr TwoWire &I2C_BUS = Wire;
/* MPU-9250 */
inline constexpr float MPU9250_MAG_FRAME_RATE_HZ = 100;
inline constexpr int8_t MPU9250_SRD = 9;
inline constexpr int8_t MPU9250_CS = 36;
inline constexpr int8_t MPU9250_DRDY = 35;
inline constexpr int32_t HEALTHY_TIMEOUT_MS = 50;
/* VectorNav */
inline constexpr int8_t VN_SRD = 3;
inline constexpr int8_t VN_CS = 34;
inline constexpr int8_t VN_DRDY = 33;
/* BME-280 */
inline constexpr int8_t BME280_CS = 32;
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
inline constexpr int8_t BATTERY_VOLTAGE_PIN = 23;
inline constexpr int8_t BATTERY_CURRENT_PIN = 22;

/* FMU-R V2-Beta */
#elif defined(__FMU_R_V2_BETA__)
/* Messages */
inline constexpr usb_serial_class &MSG_BUS = Serial;
/* Inceptor / Effector */
inline constexpr int32_t SBUS_HEALTHY_TIMEOUT_MS = 100;
inline constexpr int8_t NUM_SBUS_CH = 16;
inline constexpr int8_t NUM_PWM_PINS = 8;
inline constexpr HardwareSerial &SBUS_UART = Serial2;
inline constexpr std::array<int8_t, NUM_PWM_PINS> PWM_PINS = {37, 22, 23, 6,
                                                               5, 4, 3, 2};
/* Frame period */
inline constexpr float FRAME_RATE_HZ = 100.0f;
inline constexpr float FRAME_PERIOD_MS = 10.0f;
/* 90% of the frame period */
inline constexpr float EFFECTOR_DELAY_US = FRAME_PERIOD_MS * 0.9f * 1e3;
/* SPI BUS */
inline constexpr SPIClass &SPI_BUS = SPI;
/* I2C Bus */
inline constexpr TwoWire &I2C_BUS = Wire;
/* MPU-9250 */
inline constexpr float MPU9250_MAG_FRAME_RATE_HZ = 100;
inline constexpr int8_t MPU9250_SRD = 9;
inline constexpr int8_t MPU9250_CS = 36;
inline constexpr int8_t MPU9250_DRDY = 35;
inline constexpr int32_t HEALTHY_TIMEOUT_MS = 50;
/* VectorNav */
inline constexpr int8_t VN_SRD = 3;
inline constexpr int8_t VN_CS = 34;
inline constexpr int8_t VN_DRDY = 33;
/* BME-280 */
inline constexpr int8_t BME280_CS = 32;
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
inline constexpr int32_t SBUS_HEALTHY_TIMEOUT_MS = 100;
inline constexpr int8_t NUM_SBUS_CH = 16;
inline constexpr int8_t NUM_PWM_PINS = 8;
inline constexpr HardwareSerial &SBUS_UART = Serial2;
inline constexpr std::array<int8_t, NUM_PWM_PINS> PWM_PINS = {21, 22, 23, 2,
                                                               3, 4, 5, 6};
/* Frame period */
inline constexpr float FRAME_RATE_HZ = 50.0f;
inline constexpr float FRAME_PERIOD_MS = 20.0f;
/* 90% of the frame period */
inline constexpr float EFFECTOR_DELAY_US = FRAME_PERIOD_MS * 0.9f * 1e3;
/* SPI BUS */
inline constexpr SPIClass &SPI_BUS = SPI;
/* I2C Bus */
inline constexpr TwoWire &I2C_BUS = Wire1;
/* MPU-9250 */
inline constexpr float MPU9250_MAG_FRAME_RATE_HZ = 8;
inline constexpr int8_t MPU9250_SRD = 19;
inline constexpr int8_t MPU9250_CS = 24;
inline constexpr int8_t MPU9250_DRDY = 27;
inline constexpr int32_t HEALTHY_TIMEOUT_MS = 100;
/* VectorNav */
inline constexpr int8_t VN_SRD = 7;
inline constexpr int8_t VN_CS = 25;
inline constexpr int8_t VN_DRDY = 28;
/* BME-280 */
inline constexpr int8_t BME280_CS = 26;
/* Analog */
inline constexpr int8_t NUM_AIN_PINS = 2;
inline constexpr int8_t AIN_PINS[NUM_AIN_PINS] = {14, 16};
/* BFS Bus */
inline constexpr int8_t BFS_INT1 = 20;
inline constexpr int8_t BFS_INT2 = 17;
/* Voltage */
inline constexpr int ANALOG_RESOLUTION_BITS = 16;
inline constexpr float VOLTAGE_RANGE = 3.3;
inline constexpr float ANALOG_COUNT_RANGE =
  std::pow(2.0f, ANALOG_RESOLUTION_BITS) - 1.0f;
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

#endif  // FLIGHT_CODE_INCLUDE_FLIGHT_HARDWARE_DEFS_H_
