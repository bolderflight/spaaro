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

#ifndef INCLUDE_FLIGHT_CONFIG_H_
#define INCLUDE_FLIGHT_CONFIG_H_

#include "flight/global_defs.h"
#include "ams5915/ams5915.h"

/* Debug messages */
static constexpr bool DEBUG = true;
/* GNSS */
static constexpr HardwareSerial &GNSS_UART = Serial3;
/* Airdata */
static constexpr bool PITOT_STATIC_INSTALLED = false;
#if (PITOT_STATIC_INSTALLED)
static constexpr bfs::Ams5915Transducer STATIC_PRES_TYPE = bfs::AMS5915_1200_B;
static constexpr int8_t STATIC_PRES_ADDR = 0x10;
static constexpr bfs::Ams5915Transducer DIFF_PRES_TYPE = bfs::AMS5915_0010_D;
static constexpr int8_t DIFF_PRES_ADDR = 0x11;
#else
static constexpr int8_t STATIC_PRES_CS = 26;
#endif
/* Aircraft config */
extern AircraftConfig config;

#endif  // INCLUDE_FLIGHT_CONFIG_H_
