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

#ifndef FLIGHT_CODE_INCLUDE_FLIGHT_GLOBAL_DEFS_H_
#define FLIGHT_CODE_INCLUDE_FLIGHT_GLOBAL_DEFS_H_

#include "flight/hardware_defs.h"
#include "imu/imu.h"
#include "gnss/gnss.h"
#include "pres/pres.h"
#include "global_defs/global_defs.h"
#ifndef SIL
#include "mpu9250/mpu9250.h"
#include "ublox/ublox.h"
#include "ams5915/ams5915.h"
#include "bme280/bme280.h"
#endif
#include "sbus/sbus.h"
#ifndef SIL
#include "pwm/pwm.h"
#include "units/units.h"
#endif
/* Control sizes */
inline constexpr std::size_t NUM_AUX_VAR = 24;
/* Telem sizes */
inline constexpr std::size_t NUM_TELEM_PARAMS = 24;
#if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__)
inline constexpr std::size_t NUM_FLIGHT_PLAN_POINTS = 500;
inline constexpr std::size_t NUM_FENCE_POINTS = 100;
inline constexpr std::size_t NUM_RALLY_POINTS = 10;
#endif
#if defined(__FMU_R_V1__)
inline constexpr std::size_t NUM_FLIGHT_PLAN_POINTS = 100;
inline constexpr std::size_t NUM_FENCE_POINTS = 50;
inline constexpr std::size_t NUM_RALLY_POINTS = 10;
#endif
/* Sensor config */
struct SensorConfig {
  bool pitot_static_installed;
  bfs::ImuConfig imu;
  bfs::GnssConfig gnss;
  bfs::PresConfig static_pres;
  bfs::PresConfig diff_pres;
};
/* Nav config */
struct NavConfig {
  float accel_cutoff_hz;
  float gyro_cutoff_hz;
  float mag_cutoff_hz;
  float static_pres_cutoff_hz;
  float diff_pres_cutoff_hz;
};
/* Telem config */
struct TelemConfig {
  bfs::AircraftType aircraft_type;
    #ifndef SIL
  HardwareSerial *bus;
    #endif
  int32_t baud;
};
/* Aircraft config */
struct AircraftConfig {
  SensorConfig sensor;
  NavConfig nav;
  TelemConfig telem;
};
/* System data */
struct SysData {
  int32_t frame_time_us;
  #if defined(__FMU_R_V1__)
  float input_volt;
  float reg_volt;
  float pwm_volt;
  float sbus_volt;
  #endif
  int64_t sys_time_us;
};
/* Analog data */
struct AdcData {
  std::array<float, NUM_AIN_PINS> volt;
};
/* Power module data */
#if defined(__FMU_R_V2__)
struct PowerModuleData {
  float voltage_v;
  float current_v;
};
#endif
/* Inceptor data */
struct InceptorData {
  bool new_data;
  bool lost_frame;
  bool failsafe;
  bool ch17;
  bool ch18;
  std::array<int16_t, bfs::SbusRx::NUM_CH> ch;
};
/* Sensor data */
struct SensorData {
  bool pitot_static_installed;
  InceptorData inceptor;
  bfs::ImuData imu;
  bfs::GnssData gnss;
  bfs::PresData static_pres;
  bfs::PresData diff_pres;
  AdcData adc;
  #if defined(__FMU_R_V2__)
  PowerModuleData power_module;
  #endif
};
/* Nav data */
struct NavData {
  bool nav_initialized;
  float pitch_rad;
  float roll_rad;
  float heading_rad;
  float alt_wgs84_m;
  float home_alt_wgs84_m;
  float alt_msl_m;
  float alt_rel_m;
  float static_pres_pa;
  float diff_pres_pa;
  float alt_pres_m;
  float ias_mps;
  float gnd_spd_mps;
  float gnd_track_rad;
  float flight_path_rad;
  std::array<float, 3> accel_bias_mps2;
  std::array<float, 3> gyro_bias_radps;
  std::array<float, 3> accel_mps2;
  std::array<float, 3> gyro_radps;
  std::array<float, 3> mag_ut;
  std::array<float, 3> ned_pos_m;
  std::array<float, 3> ned_vel_mps;
  double lat_rad;
  double lon_rad;
  double home_lat_rad;
  double home_lon_rad;
};
/* SBUS data */
struct SbusCmd {
  bool ch17;
  bool ch18;
  std::array<int16_t, NUM_SBUS_CH> cnt;
  std::array<float, NUM_SBUS_CH> cmd;
};
/* PWM data */
struct PwmCmd {
  std::array<int16_t, NUM_PWM_PINS> cnt;
  std::array<float, NUM_PWM_PINS> cmd;
};
/* Analog data */
struct AnalogData {
  std::array<float, NUM_AIN_PINS> val;
};
#if defined(__FMU_R_V2__)
/* Battery data */
struct BatteryData {
  float voltage_v;
  float current_ma;
  float consumed_mah;
  float remaining_prcnt;
  float remaining_time_s;
};
#endif
/* VMS data */
struct VmsData {
  bool motors_enabled;
  bool waypoint_reached;
  int8_t mode;
  float throttle_cmd_prcnt;
  std::array<float, NUM_AUX_VAR> aux;
  SbusCmd sbus;
  PwmCmd pwm;
  AnalogData analog;
  #if defined(__FMU_R_V2__)
  BatteryData battery;
  #endif
};
/* Telemetry data */
struct TelemData {
  bool waypoints_updated;
  bool fence_updated;
  bool rally_points_updated;
  int16_t current_waypoint;
  int16_t num_waypoints;
  int16_t num_fence_items;
  int16_t num_rally_points;
  std::array<float, NUM_TELEM_PARAMS> param;
  std::array<bfs::MissionItem, NUM_FLIGHT_PLAN_POINTS> flight_plan;
  std::array<bfs::MissionItem, NUM_FENCE_POINTS> fence;
  std::array<bfs::MissionItem, NUM_RALLY_POINTS> rally;
};
/* Aircraft data */
struct AircraftData {
  SysData sys;
  SensorData sensor;
  NavData nav;
  VmsData vms;
  TelemData telem;
};

#endif  // FLIGHT_CODE_INCLUDE_FLIGHT_GLOBAL_DEFS_H_
