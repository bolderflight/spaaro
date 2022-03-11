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

#ifndef FLIGHT_CODE_INCLUDE_FLIGHT_SENSORS_H_
#define FLIGHT_CODE_INCLUDE_FLIGHT_SENSORS_H_

#include "core/core.h"
#include <variant>
#include <optional>

/* Supported sensors */
enum SensorType : int8_t {
  NONE,
  MPU9250,
  VN100,
  VN200,
  VN300,
  BME280,
  AMS5915,
  UBLOX
};
/* Sensor configuration */
struct SensorConfig {
  SensorType type;
  std::optional<int8_t> transducer;
  std::optional<int8_t> addr;
  std::optional<int8_t> cs;
  std::optional<int32_t> baud;
  std::variant<TwoWire *, SPIClass *> bus;
};
/* Common sensor interface */
class SensorIface {
 public:
  virtual bool Init(const SensorConfig &config) = 0;
  virtual bool Read() = 0;
};

#endif  // FLIGHT_CODE_INCLUDE_FLIGHT_SENSORS_H_
