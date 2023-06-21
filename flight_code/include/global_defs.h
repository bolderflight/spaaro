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

#ifndef FLIGHT_CODE_INCLUDE_GLOBAL_DEFS_H_
#define FLIGHT_CODE_INCLUDE_GLOBAL_DEFS_H_

#include <cstddef>
#include <cinttypes>
#include "hardware_defs.h"
#include "eigen.h"
#include "mavlink.h"
#include "ams5915.h"

/* Control sizes */
inline constexpr std::size_t NUM_AUX_VAR = 24;
/* Telem sizes */
inline constexpr std::size_t NUM_UTM_MSG = 1;
inline constexpr std::size_t NUM_TELEM_PARAMS = 24;
#if defined(__FMU_R_MINI_V1__) || defined(__FMU_R_V2__) || \
    defined(__FMU_R_V2_BETA__)
inline constexpr std::size_t NUM_FLIGHT_PLAN_POINTS = 500;
inline constexpr std::size_t NUM_FENCE_POINTS = 100;
inline constexpr std::size_t NUM_RALLY_POINTS = 10;
#else
inline constexpr std::size_t NUM_FLIGHT_PLAN_POINTS = 100;
inline constexpr std::size_t NUM_FENCE_POINTS = 50;
inline constexpr std::size_t NUM_RALLY_POINTS = 10;
#endif

enum DlpfBandwidth : int8_t {
  #if defined(__FMU_R_MINI_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  DLPF_BANDWIDTH_92HZ,
  DLPF_BANDWIDTH_41HZ,
  #endif
  DLPF_BANDWIDTH_20HZ,
  DLPF_BANDWIDTH_10HZ,
  DLPF_BANDWIDTH_5HZ
};

struct FmuConfig {
  DlpfBandwidth dlpf_hz = DLPF_BANDWIDTH_41HZ;
  float accel_bias_mps[3] = {0, 0, 0};
  float mag_bias_ut[3] = {0, 0, 0};
  float accel_scale[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  float mag_scale[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  float rotation[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};

enum VectorNavDevice : int8_t {
  VECTOR_NAV_NONE,
  VECTOR_NAV_VN100,
  VECTOR_NAV_VN200,
  VECTOR_NAV_VN300
};

struct VectorNavConfig {
  VectorNavDevice device = VECTOR_NAV_NONE;
  uint16_t accel_filt_window = 4;
  uint16_t gyro_filt_window = 4;
  uint16_t mag_filt_window = 0;
  uint16_t temp_filt_window = 4;
  uint16_t pres_filt_window = 0;
  float antenna_offset_m[3] = {0, 0, 0};
  float antenna_baseline_m[3] = {0, 0, 0};
  float baseline_uncertainty_m[3] = {0, 0, 0};
  float rotation[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};

enum ExtMag : int8_t {
  EXT_MAG_NONE,
  EXT_MAG_PRIM,
  EXT_MAG_SEC
};

struct MagConfig {
  ExtMag device = EXT_MAG_NONE;
  float mag_bias_ut[3] = {0, 0, 0};
  float mag_scale[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  float rotation[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};

struct GnssConfig {
  int32_t baud = -1;
};

enum PresTransducer : int8_t {
  PRES_NONE = -1,
  PRES_AMS5915_0005_D = bfs::Ams5915::AMS5915_0005_D,
  PRES_AMS5915_0010_D = bfs::Ams5915::AMS5915_0010_D,
  PRES_AMS5915_0005_D_B = bfs::Ams5915::AMS5915_0005_D_B,
  PRES_AMS5915_0010_D_B = bfs::Ams5915::AMS5915_0010_D_B,
  PRES_AMS5915_0020_D = bfs::Ams5915::AMS5915_0020_D,
  PRES_AMS5915_0050_D = bfs::Ams5915::AMS5915_0050_D,
  PRES_AMS5915_0100_D = bfs::Ams5915::AMS5915_0100_D,
  PRES_AMS5915_0020_D_B = bfs::Ams5915::AMS5915_0020_D_B,
  PRES_AMS5915_0050_D_B = bfs::Ams5915::AMS5915_0050_D_B,
  PRES_AMS5915_0100_D_B = bfs::Ams5915::AMS5915_0100_D_B,
  PRES_AMS5915_0200_D = bfs::Ams5915::AMS5915_0200_D,
  PRES_AMS5915_0350_D = bfs::Ams5915::AMS5915_0350_D,
  PRES_AMS5915_1000_D = bfs::Ams5915::AMS5915_1000_D,
  PRES_AMS5915_2000_D = bfs::Ams5915::AMS5915_2000_D,
  PRES_AMS5915_4000_D = bfs::Ams5915::AMS5915_4000_D,
  PRES_AMS5915_7000_D = bfs::Ams5915::AMS5915_7000_D,
  PRES_AMS5915_10000_D = bfs::Ams5915::AMS5915_10000_D,
  PRES_AMS5915_0200_D_B = bfs::Ams5915::AMS5915_0200_D_B,
  PRES_AMS5915_0350_D_B = bfs::Ams5915::AMS5915_0350_D_B,
  PRES_AMS5915_1000_D_B = bfs::Ams5915::AMS5915_1000_D_B,
  PRES_AMS5915_1000_A = bfs::Ams5915::AMS5915_1000_A,
  PRES_AMS5915_1200_B = bfs::Ams5915::AMS5915_1200_B
};

struct  PresConfig {
  uint8_t addr;
  PresTransducer device = PRES_NONE;
};

enum RadAlt {
  RAD_ALT_NONE,
  RAD_ALT_AINSTEIN_USD1
};

struct RadAltConfig {
  RadAlt device = RAD_ALT_NONE;
};

enum OpFlow {
  OPFLOW_NONE,
  OPFLOW_MATEK3901
};

struct OpFlowConfig{
  OpFlow device = OPFLOW_NONE;
};

#if defined(__FMU_R_MINI_V1__) || defined(__FMU_R_V2__)
struct PowerModuleConfig {
  float volts_per_volt = 15.3f;
  float amps_per_volt = 50.0f;
};
#endif

struct SensorConfig {
  FmuConfig fmu;
  MagConfig ext_mag;
  GnssConfig ext_gnss1;
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  GnssConfig ext_gnss2;
  #endif
  PresConfig ext_pres1;
  PresConfig ext_pres2;
  PresConfig ext_pres3;
  PresConfig ext_pres4;
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  OpFlowConfig opflow;
  RadAltConfig rad_alt;
  #endif
  #if defined(__FMU_R_MINI_V1__) || defined(__FMU_R_V2__)
  PowerModuleConfig power_module;
  #endif
};

enum InsImuSource : int8_t {
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  INS_IMU_VECTOR_NAV,
  #endif
  INS_IMU_FMU
};

enum InsMagSource : int8_t {
  INS_MAG_FMU,
  INS_MAG_EXT_MAG,
};

enum InsGnssSource : int8_t {
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  INS_GNSS_VECTOR_NAV,
  #endif
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  INS_GNSS_EXT_GNSS2,
  #endif
  INS_GNSS_EXT_GNSS1
};

struct InsConfig {
  InsImuSource imu_source = INS_IMU_FMU;
  InsMagSource mag_source = INS_MAG_FMU;
  InsGnssSource gnss_source = INS_GNSS_EXT_GNSS1;
  float accel_cutoff_hz = 40;
  float gyro_cutoff_hz = 40;
  float mag_cutoff_hz = 10;
  float hardcoded_heading = -1.0f;
  Eigen::Vector3f antenna_baseline_m = Eigen::Vector3f::Zero();
};

enum AuxInsSource {
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  AUX_INS_VECTOR_NAV,
  #endif
  AUX_INS_BFS
};

struct AuxInsConfig {
  AuxInsSource ins_source = AUX_INS_BFS;
};

enum AdcStaticPresSource : int8_t {
  ADC_STATIC_PRES_FMU,
  ADC_STATIC_PRES_EXT_PRES1,
  ADC_STATIC_PRES_EXT_PRES2,
  ADC_STATIC_PRES_EXT_PRES3,
  ADC_STATIC_PRES_EXT_PRES4
};

enum AdcDiffPresSource : int8_t {
  ADC_DIFF_PRES_NONE,
  ADC_DIFF_PRES_EXT_PRES1,
  ADC_DIFF_PRES_EXT_PRES2,
  ADC_DIFF_PRES_EXT_PRES3,
  ADC_DIFF_PRES_EXT_PRES4
};

struct AdcConfig {
  AdcStaticPresSource static_pres_source = ADC_STATIC_PRES_FMU;
  AdcDiffPresSource diff_pres_source = ADC_DIFF_PRES_NONE;
  float static_pres_cutoff_hz = 10;
  float diff_pres_cutoff_hz = 10;
};

enum TelemImuSource : int8_t {
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  TELEM_IMU_VECTOR_NAV,
  #endif
  TELEM_IMU_FMU
};

enum TelemMagSource : int8_t {
  TELEM_MAG_FMU,
  TELEM_MAG_EXT_MAG
};

enum TelemGnssSource : int8_t {
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  TELEM_GNSS_VECTOR_NAV,
  #endif
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  TELEM_GNSS_EXT_GNSS2,
  #endif
  TELEM_GNSS_EXT_GNSS1
};

enum TelemStaticPresSource : int8_t {
  TELEM_STATIC_PRES_FMU,
  TELEM_STATIC_PRES_EXT_PRES1,
  TELEM_STATIC_PRES_EXT_PRES2,
  TELEM_STATIC_PRES_EXT_PRES3,
  TELEM_STATIC_PRES_EXT_PRES4
};

enum TelemDiffPresSource : int8_t {
  TELEM_DIFF_PRES_NONE,
  TELEM_DIFF_PRES_EXT_PRES1,
  TELEM_DIFF_PRES_EXT_PRES2,
  TELEM_DIFF_PRES_EXT_PRES3,
  TELEM_DIFF_PRES_EXT_PRES4
};

enum TelemInsSource : int8_t {
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  TELEM_INS_VECTOR_NAV,
  #endif
  TELEM_INS_BFS
};

enum TelemGnssRtk : int8_t {
  TELEM_GNSS_RTK_NONE,
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  TELEM_GNSS_RTK_EXT_GNSS2,
  #endif
  TELEM_GNSS_RTK_EXT_GNSS1
};

struct TelemConfig {
  int32_t baud = 57600;
  bfs::AircraftType aircraft_type = bfs::FIXED_WING;
  TelemImuSource imu_source = TELEM_IMU_FMU;
  TelemMagSource mag_source = TELEM_MAG_FMU;
  TelemGnssSource gnss_source = TELEM_GNSS_EXT_GNSS1;
  TelemStaticPresSource static_pres_source = TELEM_STATIC_PRES_FMU;
  TelemDiffPresSource diff_pres_source = TELEM_DIFF_PRES_NONE;
  TelemInsSource ins_source = TELEM_INS_BFS;
  TelemGnssRtk gnss_rtk = TELEM_GNSS_RTK_NONE;
  int16_t raw_sens_stream_preiod_ms = 500;
  int16_t ext_status_stream_period_ms = 1000;
  int16_t rc_channel_stream_period_ms = 500;
  int16_t pos_stream_period_ms = 250;
  int16_t extra1_stream_period_ms = 100;
  int16_t extra2_stream_period_ms = 100;
};

struct AircraftConfig {
  SensorConfig sensor;
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  VectorNavConfig vector_nav;
  #endif
  InsConfig bfs_ins;
  AuxInsConfig aux_ins;
  AdcConfig adc;
  TelemConfig telem;
};

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

struct InceptorData {
  static constexpr int8_t max_ch = 16;
  bool new_data;
  bool lost_frame;
  bool failsafe;
  std::array<int16_t, max_ch> ch;
};

struct ImuData {
  bool installed = false;
  bool healthy;
  bool new_data;
  float die_temp_c;
  float accel_mps2[3];
  float gyro_radps[3];
};

struct MagData {
  bool installed = false;
  bool healthy;
  bool new_data;
  float die_temp_c;
  float mag_ut[3];
};

struct OpFlowData {
  bool installed = false;
  bool healthy;
  bool new_data;
  int32_t mot_x;
  int32_t mot_y;
  uint8_t sur_qual;
  int32_t range_mm;
  uint8_t range_qual;
};

struct GnssData {
  bool installed = false;
  bool healthy;
  bool new_data;
  bool rel_pos_avail;
  bool rel_pos_moving_baseline;
  bool rel_pos_baseline_normalized;
  int8_t fix;
  int8_t num_sats;
  int16_t gps_week;
  float alt_wgs84_m;
  float horz_acc_m;
  float vert_acc_m;
  float vel_acc_mps;
  float ned_vel_mps[3];
  float rel_pos_acc_ned_m[3];
  double gps_tow_s;
  double lat_rad;
  double lon_rad;
  double rel_pos_ned_m[3];
};

struct PresData {
  bool installed = false;
  bool is_static_pres = false;
  bool healthy;
  bool new_data;
  float die_temp_c;
  float pres_pa;
};

struct RadAltData {
  bool installed = false;
  bool healthy;
  bool new_data;
  uint8_t snr;
  float alt_m;
};

struct AnalogData {
  float voltage_v[NUM_AIN_PINS];
};

struct PowerModuleData {
  float voltage_v;
  float current_ma;
};

struct SensorData {
  InceptorData inceptor;
  ImuData fmu_imu;
  MagData fmu_mag;
  PresData fmu_static_pres;
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  ImuData vector_nav_imu;
  MagData vector_nav_mag;
  PresData vector_nav_static_pres;
  GnssData vector_nav_gnss;
  #endif
  MagData ext_mag;
  GnssData ext_gnss1;
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  GnssData ext_gnss2;
  #endif
  PresData ext_pres1;
  PresData ext_pres2;
  PresData ext_pres3;
  PresData ext_pres4;
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  OpFlowData opflow;
  RadAltData rad_alt;
  #endif
  AnalogData analog;
  #if defined(__FMU_R_V2__) || defined(__FMU_R_MINI_V1__)
  PowerModuleData power_module;
  #endif
};

struct InsData {
  bool initialized;
  float pitch_rad;
  float roll_rad;
  float heading_rad;
  float alt_wgs84_m;
  float accel_mps2[3];
  float gyro_radps[3];
  float mag_ut[3];
  float ned_vel_mps[3];
  double lat_rad;
  double lon_rad;
};

struct AdcData {
  float static_pres_pa;
  float diff_pres_pa;
  float pres_alt_m;
  float rel_alt_m;
  float ias_mps;
};

struct AuxInsData {
  float home_alt_wgs84_m;
  float gnd_spd_mps;
  float gnd_track_rad;
  float flight_path_rad;
  double home_lat_rad;
  double home_lon_rad;
  double ned_pos_m[3];
};

struct TelemData {
  bool waypoints_updated;
  bool fence_updated;
  bool rally_points_updated;
  int16_t current_waypoint;
  int16_t num_waypoints;
  int16_t num_fence_items;
  int16_t num_rally_points;
  std::array<float, NUM_TELEM_PARAMS> param;
  bfs::MissionItem flight_plan[NUM_FLIGHT_PLAN_POINTS];
  bfs::MissionItem fence[NUM_FENCE_POINTS];
  bfs::MissionItem rally[NUM_RALLY_POINTS];
};

struct VmsData {
  bool advance_waypoint;
  bool motors_enabled;
  int8_t mode;
  std::array<int16_t, NUM_SBUS_CH> sbus;
  std::array<int16_t, NUM_PWM_PINS> pwm;
  float throttle_cmd_prcnt;
  float flight_time_remaining_s;
  float power_remaining_prcnt;
  float aux[NUM_AUX_VAR];
};

struct StateEstData {
  InsData bfs_ins;
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  InsData vector_nav_ins;
  #endif
  AuxInsData aux_ins;
  AdcData adc;
};

struct AircraftData {
  SysData sys;
  SensorData sensor;
  StateEstData state_est;
  TelemData telem;
  VmsData vms;
};

#endif  // FLIGHT_CODE_INCLUDE_GLOBAL_DEFS_H_
