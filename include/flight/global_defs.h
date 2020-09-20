/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FLIGHT_GLOBAL_DEFS_H_
#define INCLUDE_FLIGHT_GLOBAL_DEFS_H_

#include "flight/hardware_defs.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include <array>

/* Inceptor data */
struct InceptorData {
  std::array<uint16_t, 16> cmds;
};
/* IMU data */
struct ImuData {
  Eigen::Vector3f accel_mps2;
  Eigen::Vector3f gyro_radps;
  Eigen::Vector3f mag_ut;
  float die_temp_c;
};
/* GNSS data */
struct GnssData {
  bool updated;
  int16_t year;
  int8_t month;
  int8_t day;
  int8_t hour;
  int8_t min;
  float sec;
  uint8_t fix;
  uint8_t num_satellites;
  Eigen::Vector3f ned_vel_mps;
  Eigen::Vector3d lla_msl_rad_m;
  float alt_wgs84_m;
  float ground_speed_mps;
  float ground_track_rad;
  uint32_t time_accuracy_ns;
  float horiz_accuracy_m;
  float vert_accuracy_m;
  float vel_accuracy_mps;
  float track_accuracy_rad;
};
/* EKF data */
struct EkfData {
  Eigen::Vector3f accel_bias_mps2;
  Eigen::Vector3f gyro_bias_radps;
  Eigen::Vector3f accel_mps2;
  Eigen::Vector3f gyro_radps;
  Eigen::Vector3f ned_vel_mps;
  Eigen::Vector3d lla_rad_m;
  float pitch_rad;
  float roll_rad;
  float yaw_rad;
};
/* INS data */
struct InsData {
  ImuData imu;
  GnssData gnss;
  EkfData ekf;
};
/* Presssure transducer data */
struct PressureTransducerData {
  float press_pa;
  float die_temp_c;
};
/* Airdata */
struct Airdata {
  PressureTransducerData ps_static;
  PressureTransducerData ps_diff;
  float filt_static_press_pa;
  float filt_diff_press_pa;
  float press_alt_m;
  float agl_alt_m;
  float ias_mps;
  float eas_mps;
};
/* Status data */
struct StatusData {
  float input_voltage;
  float regulated_voltage;
  float pwm_voltage;
  float sbus_voltage;
};
/* Effector Commands */
struct EffectorCmds {
  std::array<uint16_t, NUM_PWM_PINS> pwm;
  std::array<uint16_t, 16> sbus;
};
/* Controls data */
struct ControlsData {
  EffectorCmds cmds;
};
/* Aircraft data */
struct AircraftData {
  double time_s;
  InceptorData inceptor;
  InsData ins;
  Airdata airdata;
  StatusData status;
  ControlsData control;
};

#endif  // INCLUDE_FLIGHT_GLOBAL_DEFS_H_
