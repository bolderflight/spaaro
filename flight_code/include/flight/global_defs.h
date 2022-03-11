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

#ifndef FLIGHT_CODE_INCLUDE_FLIGHT_GLOBAL_DEFS_H_
#define FLIGHT_CODE_INCLUDE_FLIGHT_GLOBAL_DEFS_H_








// enum ImuType : int8_t {
//   IMU_TYPE_NONE,
//   IMU_TYPE_MPU9250,
//   IMU_TYPE_VN100,
//   IMU_TYPE_VN200,
//   IMU_TYPE_VN300
// };

// struct ImuConfig {
//   ImuType type;
//   bool primary = false;
//   int8_t dev;
//   std::variant<TwoWire *, SPIClass *> bus;
//   Eigen::Vector3f accel_bias_mps2 = Eigen::Vector3f::Zero();
//   Eigen::Matrix3f accel_scale = Eigen::Matrix3f::Identity();
//   Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
// };

// enum MagType : {
//   MAG_TYPE_NONE,
//   MAG_TYPE_MPU9250,
//   MAG_TYPE_VN100,
//   MAG_TYPE_VN200,
//   MAG_TYPE_VN300
// };

// struct MagConfig {
//   MagType type;
//   bool primary = false;
//   int8_t dev;
//   std::variant<TwoWire *, SPIClass *> bus;
//   Eigen::Vector3f mag_bias_ut = Eigen::Vector3f::Zero();
//   Eigen::Matrix3f mag_scale = Eigen::Matrix3f::Identity();
//   Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
// };

// enum StaticPresType : int8_t {
//   STATIC_PRES_NONE,
//   STATIC_PRES_BME280,
//   STATIC_PRES_AMS5915
// }

// struct StaticPresConfig {
//   StaticPresType type;
//   bool primary = false;
//   int8_t transducer;
//   int8_t dev;
//   std::variant<TwoWire *, SPIClass *> bus;
// };

// enum DiffPresType : int8_t {
//   DIFF_PRES_NONE,
//   DIFF_PRES_AMS5915
// }

// struct DiffPresConfig {
//   DiffPresType type;
//   bool primary = false;
//   int8_t transducer;
//   int8_t dev;
//   std::variant<TwoWire *, SPIClass *> bus;
// };

// enum GnssType : int8_t {

// };

#endif  // FLIGHT_CODE_INCLUDE_FLIGHT_GLOBAL_DEFS_H_
