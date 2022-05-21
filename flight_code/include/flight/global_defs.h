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

#include <Eigen/Dense>
#include "flight/hardware_defs.h"
#include "mpu9250.h"  // NOLINT
#include "mavlink.h"  // NOLINT
#include "eigen.h"  // NOLINT
#include "ms4525do.h"  // NOLINT

using namespace bfs;  // NOLINT
using namespace Eigen;  // NOLINT

/* DroneCAN sizes */
inline constexpr std::size_t NUM_DRONE_CAN_ACT_CH = 15;
inline constexpr std::size_t NUM_DRONE_CAN_ESC_CH = 20;
/* Control sizes */
inline constexpr std::size_t NUM_AUX_VAR = 24;
/* Telem sizes */
inline constexpr std::size_t NUM_TELEM_PARAMS = 24;
#if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__)
inline constexpr std::size_t NUM_FLIGHT_PLAN_POINTS = 500;
inline constexpr std::size_t NUM_FENCE_POINTS = 100;
inline constexpr std::size_t NUM_RALLY_POINTS = 10;
#else
inline constexpr std::size_t NUM_FLIGHT_PLAN_POINTS = 100;
inline constexpr std::size_t NUM_FENCE_POINTS = 50;
inline constexpr std::size_t NUM_RALLY_POINTS = 10;
#endif

enum DrdySources : int8_t {
  DRDY_MPU9250,
  DRDY_VECTORNAV
};

struct Mpu9250Config {
  Mpu9250::AccelRange accel_range_g = Mpu9250::ACCEL_RANGE_16G;
  Mpu9250::GyroRange gyro_range_dps = Mpu9250::GYRO_RANGE_2000DPS;
  Mpu9250::DlpfBandwidth dlpf_hz = Mpu9250::DLPF_BANDWIDTH_20HZ;
  Vector3f accel_bias_mps2 = Vector3f::Zero();
  Vector3f mag_bias_ut = Vector3f::Zero();
  Matrix3f accel_scale = Matrix3f::Identity();
  Matrix3f mag_scale = Matrix3f::Identity();
  Matrix3f rotation = Matrix3f::Identity();
};

enum VectorNavDevice : int8_t {
  VECTORNAV_NONE,
  VECTORNAV_VN100,
  VECTORNAV_VN200,
  VECTORNAV_VN300
};

struct VnConfig {
  VectorNavDevice device = VECTORNAV_NONE;
  uint16_t accel_filt_window = 4;
  uint16_t gyro_filt_window = 4;
  uint16_t mag_filt_window = 0;
  uint16_t temp_filt_window = 4;
  uint16_t pres_filt_window = 0;
  Vector3f antenna_offset_m = Vector3f::Zero();
  Vector3f antenna_baseline_m = Vector3f::Zero();
  Vector3f baseline_uncertainty_m = Vector3f::Zero();
  Matrix3f rotation = Matrix3f::Identity();
};

enum Ams5915Transducer : int8_t {
  AMS5915_NONE = -1,
  AMS5915_0005_D,
  AMS5915_0010_D,
  AMS5915_0005_D_B,
  AMS5915_0010_D_B,
  AMS5915_0020_D,
  AMS5915_0050_D,
  AMS5915_0100_D,
  AMS5915_0020_D_B,
  AMS5915_0050_D_B,
  AMS5915_0100_D_B,
  AMS5915_0200_D,
  AMS5915_0350_D,
  AMS5915_1000_D,
  AMS5915_2000_D,
  AMS5915_4000_D,
  AMS5915_7000_D,
  AMS5915_10000_D,
  AMS5915_0200_D_B,
  AMS5915_0350_D_B,
  AMS5915_1000_D_B,
  AMS5915_1000_A,
  AMS5915_1200_B
};

struct Ams5915Config {
  uint8_t addr;
  Ams5915Transducer transducer = AMS5915_NONE;
};

struct Ms4525doConfig {
  uint8_t addr = 0;
  float min_pres;
  float max_pres;
  Ms4525do::OutputType output_type = Ms4525do::OUTPUT_TYPE_A;
};

struct GnssConfig {
  int32_t baud = -1;
};

struct SbusRxConfig {
  bool installed = false;
};

struct SensorConfig {
  DrdySources drdy_source = DRDY_MPU9250;
  SbusRxConfig sbus;
  Mpu9250Config mpu9250;
  VnConfig vector_nav;
  Ams5915Config ams5915_static_pres;
  Ams5915Config ams5915_diff_pres;
  Ms4525doConfig ms4525do;
  GnssConfig gnss_uart3;
  GnssConfig gnss_uart4;
};

enum AirDataStaticPresSource : int8_t {
  AIR_DATA_STATIC_PRES_BME280,
  AIR_DATA_STATIC_PRES_AMS5915,
  AIR_DATA_STATIC_PRES_VECTORNAV
};

enum AirDataDiffPresSource : int8_t {
  AIR_DATA_DIFF_PRES_NONE,
  AIR_DATA_DIFF_PRES_AMS5915,
  AIR_DATA_DIFF_PRES_MS4525DO
};

struct AirDataConfig {
  AirDataStaticPresSource static_pres_source = AIR_DATA_STATIC_PRES_BME280;
  AirDataDiffPresSource diff_pres_source = AIR_DATA_DIFF_PRES_NONE;
  float static_pres_cutoff_hz = 5;
  float diff_pres_cutoff_hz = 5;
};

enum EkfImuSource : int8_t {
  EKF_IMU_MPU9250,
  EKF_IMU_VECTORNAV
};

enum EkfGnssSource : int8_t {
  EKF_GNSS_UBLOX3,
  EKF_GNSS_UBLOX4,
  EKF_GNSS_VECTORNAV
};

struct BfsNavConfig {
  EkfImuSource imu_source = EKF_IMU_MPU9250;
  EkfGnssSource gnss_source_prim = EKF_GNSS_UBLOX3;
  // EkfGnssSource gnss_source_sec = EKF_GNSS_UBLOX4;
  float accel_cutoff_hz = 10;
  float gyro_cutoff_hz = 10;
  float mag_cutoff_hz = 10;
  // Vector3f antenna_baseline_m = Vector3f::Zero();
};

enum TelemImu : int8_t {
  TELEM_IMU_MPU9250,
  TELEM_IMU_VECTORNAV
};

enum TelemGnss : int8_t {
  TELEM_GNSS_UBLOX3,
  TELEM_GNSS_UBLOX4,
  TELEM_GNSS_VECTORNAV
};

enum TelemNav : int8_t {
  TELEM_NAV_BFS_EKF,
  TELEM_NAV_VECTORNAV
};

enum TelemStaticPres : int8_t {
  TELEM_STATIC_PRES_BME280,
  TELEM_STATIC_PRES_VECTORNAV,
  TELEM_STATIC_PRES_AMS5915
};

enum TelemDiffPres : int8_t {
  TELEM_DIFF_PRES_NONE,
  TELEM_DIFF_PRES_AMS5915,
  TELEM_DIFF_PRES_MS4525DO
};

struct TelemConfig {
  bfs::AircraftType aircraft_type = FIXED_WING;
  TelemImu imu_source = TELEM_IMU_MPU9250;
  TelemStaticPres static_pres_source = TELEM_STATIC_PRES_BME280;
  TelemDiffPres diff_pres_source = TELEM_DIFF_PRES_NONE;
  TelemGnss gnss_source = TELEM_GNSS_UBLOX3;
  TelemNav nav_source = TELEM_NAV_BFS_EKF;
  HardwareSerial *bus = &Serial5;
  HardwareSerial *rtk_uart = &Serial3;
  int32_t baud = 57600;
};

struct DroneCanActConfig {
  uint8_t actuator_id;
  uint8_t command_type;
};

struct DroneCanConfig {
  uint32_t baud = 1000000;
  uint8_t num_actuators;
  uint8_t num_esc;
  DroneCanActConfig config[NUM_DRONE_CAN_ACT_CH];
};

struct AircraftConfig {
  SensorConfig sensor;
  AirDataConfig airdata;
  BfsNavConfig bfs_ekf;
  DroneCanConfig drone_can;
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

struct AdcData {
  std::array<float, NUM_AIN_PINS> volt;
};

#if defined(__FMU_R_V2__)
struct PowerModuleData {
  float voltage_v;
  float current_v;
};
#endif

struct InceptorData {
  bool installed;
  bool healthy;
  bool new_data;
  bool ch17;
  bool ch18;
  std::array<int16_t, NUM_SBUS_CH> ch;
};

struct ImuData {
  bool installed;
  bool healthy;
  bool new_imu_data;
  bool new_mag_data;
  float die_temp_c;
  float accel_mps2[3];
  float gyro_radps[3];
  float mag_ut[3];
};

struct PresData {
  bool installed;
  bool healthy;
  bool new_data;
  float die_temp_c;
  float pres_pa;
};

struct GnssData {
  bool installed;
  bool healthy;
  bool new_data;
  int8_t fix;
  int8_t num_sats;
  int16_t gps_week;
  float alt_wgs84_m;
  float horz_acc_m;
  float vert_acc_m;
  float vel_acc_mps;
  float ned_vel_mps[3];
  double gps_tow_s;
  double lat_rad;
  double lon_rad;
};

struct GnssRelPosData {
  bool avail;
  bool moving_baseline;
  bool heading_valid;
  bool baseline_normalized;
  float baseline_len_acc_m;
  float heading_rad;
  float heading_acc_rad;
  double baseline_len_m;
  float rel_pos_acc_ned_m[3];
  double rel_pos_ned_m[3];
};

struct SensorData {
  InceptorData sbus_inceptor;
  ImuData mpu9250_imu;
  ImuData vector_nav_imu;
  PresData bme280_static_pres;
  PresData vector_nav_static_pres;
  PresData ams5915_static_pres;
  PresData ams5915_diff_pres;
  PresData ms4525do_diff_pres;
  GnssData vector_nav_gnss;
  GnssData ublox3_gnss;
  GnssData ublox4_gnss;
  GnssRelPosData ublox3_relpos;
  GnssRelPosData ublox4_relpos;
  AdcData adc;
  #if defined(__FMU_R_V2__)
  PowerModuleData power_module;
  #endif
};

struct AirData {
  bool initialized;
  float static_pres_pa;
  float diff_pres_pa;
  float alt_pres_m;
  float ias_mps;
};

struct InertialData {
  bool healthy;
  float pitch_rad;
  float roll_rad;
  float heading_rad;
  float alt_wgs84_m;
  std::array<float, 3> accel_mps2;
  std::array<float, 3> gyro_radps;
  std::array<float, 3> mag_ut;
  std::array<float, 3> ned_vel_mps;
  double lat_rad;
  double lon_rad;
};

struct NavData {
  AirData airdata;
  InertialData bfs_ekf;
  InertialData vector_nav;
};

struct DroneCanActCmd {
  std::array<float, NUM_DRONE_CAN_ACT_CH> cnt;
  std::array<float, NUM_DRONE_CAN_ACT_CH> cmd;
};

struct DroneCanEscCmd {
  std::array<float, NUM_DRONE_CAN_ESC_CH> cnt;
  std::array<float, NUM_DRONE_CAN_ESC_CH> cmd;
};

struct SbusCmd {
  bool ch17;
  bool ch18;
  std::array<int16_t, NUM_SBUS_CH> cnt;
  std::array<float, NUM_SBUS_CH> cmd;
};

struct PwmCmd {
  std::array<int16_t, NUM_PWM_PINS> cnt;
  std::array<float, NUM_PWM_PINS> cmd;
};

struct AnalogData {
  std::array<float, NUM_AIN_PINS> val;
};

#if defined(__FMU_R_V2__)
struct BatteryData {
  float voltage_v;
  float current_ma;
  float consumed_mah;
  float remaining_prcnt;
  float remaining_time_s;
};
#endif

struct VmsData {
  bool motors_enabled;
  bool waypoint_reached;
  int8_t mode;
  float throttle_cmd_prcnt;
  std::array<float, NUM_AUX_VAR> aux;
  SbusCmd sbus;
  PwmCmd pwm;
  DroneCanActCmd drone_can_act;
  DroneCanEscCmd drone_can_esc;
  AnalogData analog;
  #if defined(__FMU_R_V2__)
  BatteryData battery;
  #endif
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
  std::array<bfs::MissionItem, NUM_FLIGHT_PLAN_POINTS> flight_plan;
  std::array<bfs::MissionItem, NUM_FENCE_POINTS> fence;
  std::array<bfs::MissionItem, NUM_RALLY_POINTS> rally;
};

struct AircraftData {
  SysData sys;
  SensorData sensor;
  NavData nav;
  VmsData vms;
  TelemData telem;
};

#endif  // FLIGHT_CODE_INCLUDE_FLIGHT_GLOBAL_DEFS_H_
