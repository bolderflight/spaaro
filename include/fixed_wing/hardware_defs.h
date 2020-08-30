/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FIXED_WING_HARDWARE_DEFS_H_
#define INCLUDE_FIXED_WING_HARDWARE_DEFS_H_

#include "core/core.h"
#include "ams5915/ams5915.h"
#include <array>

/* Software Version */
static constexpr unsigned int MAJOR_VERSION = 1;
static constexpr unsigned int MINOR_VERSION = 0;
static constexpr unsigned int FIX_VERSION = 0;
/* Messages */    
static constexpr usb_serial_class &MSG_BUS = Serial;
/* Frame rate */
static constexpr int FRAME_RATE_HZ = 50;
static constexpr float FRAME_PERIOD_S = 1.0f / FRAME_RATE_HZ;
/* Inceptor / Effector */
static constexpr HardwareSerial &SBUS_UART = Serial2;
static constexpr int NUM_PWM_PINS = 8;
static constexpr std::array<int, NUM_PWM_PINS> PWM_PINS = {21, 22, 23, 2, 3, 4, 5, 6};
static constexpr float EFFECTOR_DELAY_US = FRAME_PERIOD_S * 0.9f * 1e6; // 90% of the frame period
/* IMU */
static constexpr SPIClass &IMU_SPI_BUS = SPI;
static constexpr unsigned int IMU_CS = 24;
static constexpr unsigned int IMU_DRDY = 27;
/* GNSS */
static constexpr HardwareSerial &GNSS_UART = Serial3;
static constexpr unsigned int GNSS_BAUD = 921600;
/* Pressure Transducers */
static constexpr i2c_t3 &STATIC_PRESS_I2C_BUS = Wire1;
static constexpr unsigned int STATIC_PRESS_ADDR = 0x10;
static constexpr sensors::Ams5915::Transducer STATIC_PRESS_TRANSDUCER = sensors::Ams5915::AMS5915_1200_B;
static constexpr i2c_t3 &DIFF_PRESS_I2C_BUS = Wire1;
static constexpr unsigned int DIFF_PRESS_ADDR = 0x11;
static constexpr sensors::Ams5915::Transducer DIFF_PRESS_TRANSDUCER   = sensors::Ams5915::AMS5915_0010_D;
static constexpr SPIClass &BME_SPI_BUS = SPI;
static constexpr unsigned int BME_CS = 26;

#endif  // INCLUDE_FIXED_WING_HARDWARE_DEFS_H_
