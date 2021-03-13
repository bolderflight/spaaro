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

#ifndef INCLUDE_FLIGHT_GLOBAL_DEFS_H_
#define INCLUDE_FLIGHT_GLOBAL_DEFS_H_

#include <cstdint>
#include <array>
#include "flight/hardware_defs.h"

static constexpr int MAJOR_VERSION = 1;
static constexpr int MINOR_VERSION = 0;
static constexpr int FIX_VERSION = 0;

struct SysMonData {
  int32_t frame_time_us;
  float input_volt;
  float reg_volt;
  float pwm_volt;
  float sbus_volt;
  double sys_time_s;
};

struct InceptorData {
  bool new_data;
  int8_t mode0;
  int8_t mode1;
  bool throttle_en;
  bool lost_frame;
  bool failsafe;
  float throttle;
  float pitch;
  float roll;
  float yaw;
};

struct ImuData {
  bool new_data;
  float accel_x_mps2;
  float accel_y_mps2;
  float accel_z_mps2;
  float gyro_x_radps;
  float gyro_y_radps;
  float gyro_z_radps;
  float mag_x_ut;
  float mag_y_ut;
  float mag_z_ut;
  struct {
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
    float gyro_x_radps;
    float gyro_y_radps;
    float gyro_z_radps;
    float mag_x_ut;
    float mag_y_ut;
    float mag_z_ut;
  } dlpf;
  float die_temp_c;
};

struct GnssData {
  bool new_data;
  int8_t fix;
  int8_t num_sats;
  int16_t week;
  float alt_wgs84_m;
  float north_vel_mps;
  float east_vel_mps;
  float down_vel_mps;
  float horz_acc_m;
  float vert_acc_m;
  float vel_acc_mps;
  double lat_rad;
  double lon_rad;
  double tow_s;
};

struct PresData {
  bool new_data;
  float pres_pa;
  struct {
    float pres_pa;
  } dlpf;
  float die_temp_c;
};

struct AirdataData {
  PresData static_pres;
  PresData diff_pres;
};

struct SensorData {
  ImuData imu;
  GnssData gnss;
  AirdataData airdata;
};

struct GuidanceData {};

struct NavData {
  float accel_x_mps2;
  float accel_y_mps2;
  float accel_z_mps2;
  float gyro_x_radps;
  float gyro_y_radps;
  float gyro_z_radps;
  float pitch_rad;
  float roll_rad;
  float heading_rad;
  float alt_wgs84_m;
  float alt_pres_m;
  float ias_mps;
  float eas_mps;
  float gnd_spd_mps;
  float gnd_track_rad;
  float flight_path_rad;
  float north_vel_mps;
  float east_vel_mps;
  float down_vel_mps;
  double lat_rad;
  double lon_rad;
};

struct ControlData {
  std::array<float, 24> aux;
};

struct EffectorCmds {
  std::array<float, NUM_PWM_PINS> pwm;
  std::array<float, NUM_SBUS_CH> sbus;
};

struct AircraftData {
  SysMonData sys_mon;
  InceptorData inceptor;
  SensorData sensor;
  GuidanceData guidance;
  NavData nav;
  ControlData control;
  EffectorCmds effector;
};

#endif  // INCLUDE_FLIGHT_GLOBAL_DEFS_H_
