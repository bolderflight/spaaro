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

#include "flight/datalog.h"
#include "flight/msg.h"
#include "logger/logger.h"
#include "framing.h"  // NOLINT
#include "./pb_encode.h"
#include "./pb_decode.h"
#if defined(__FMU_R_V2__)
#include "./datalog_fmu_v2.pb.h"
#endif
#if defined(__FMU_R_V2_BETA__)
#include "./datalog_fmu_v2_beta.pb.h"
#endif
#if defined(__FMU_R_V1__)
#include "./datalog_fmu_v1.pb.h"
#endif

namespace {
/* Datalog file name */
static const char * DATA_LOG_NAME_ = "flight_data";
/* SD card */
SdFat32 sd_;
/* Logger object */
bfs::Logger<400> logger_(&sd_);
/* Framing */
bfs::FrameEncoder<DatalogMessage_size> encoder;
/* nanopb buffer for encoding */
uint8_t data_buffer_[DatalogMessage_size];
pb_ostream_t stream_;
/* Datalog message from protobuf */
DatalogMessage datalog_msg_;
}  // namespace

void DatalogInit() {
  MsgInfo("Initializing datalog...");
  /* Initialize SD card */
  sd_.begin(SdioConfig(FIFO_SDIO));
  /* Initialize logger */
  int file_num = logger_.Init(DATA_LOG_NAME_);
  if (file_num < 0) {
    MsgError("Unable to initialize datalog.");
  }
  MsgInfo("done.\n");
}
void DatalogAdd(const AircraftData &ref) {
  /* Assign to message */
  /* sys */
  datalog_msg_.sys_frame_time_us = ref.sys.frame_time_us;
  #if defined(__FMU_R_V1__)
  datalog_msg_.sys_input_volt = ref.sys.input_volt;
  datalog_msg_.sys_reg_volt = ref.sys.reg_volt;
  datalog_msg_.sys_pwm_volt = ref.sys.pwm_volt;
  datalog_msg_.sys_sbus_volt = ref.sys.sbus_volt;
  #endif
  datalog_msg_.sys_time_s = static_cast<double>(ref.sys.sys_time_us) / 1e6;
  /* inceptor */
  datalog_msg_.incept_installed = ref.sensor.sbus_inceptor.installed;
  datalog_msg_.incept_healthy = ref.sensor.sbus_inceptor.healthy;
  datalog_msg_.incept_new_data = ref.sensor.sbus_inceptor.new_data;
  datalog_msg_.incept_ch17 = ref.sensor.sbus_inceptor.ch17;
  datalog_msg_.incept_ch18 = ref.sensor.sbus_inceptor.ch18;
  for (std::size_t i = 0; i < NUM_SBUS_CH; i++) {
    datalog_msg_.incept_ch[i] = ref.sensor.sbus_inceptor.ch[i];
  }
  /* MPU-9250 imu */
  datalog_msg_.mpu9250_imu_installed = ref.sensor.mpu9250_imu.installed;
  datalog_msg_.mpu9250_imu_healthy = ref.sensor.mpu9250_imu.healthy;
  datalog_msg_.mpu9250_imu_new_imu_data = ref.sensor.mpu9250_imu.new_imu_data;
  datalog_msg_.mpu9250_imu_new_mag_data = ref.sensor.mpu9250_imu.new_mag_data;
  datalog_msg_.mpu9250_imu_die_temp_c = ref.sensor.mpu9250_imu.die_temp_c;
  for (std::size_t i = 0; i < 3; i++) {
    datalog_msg_.mpu9250_imu_accel_mps2[i] =
      ref.sensor.mpu9250_imu.accel_mps2[i];
    datalog_msg_.mpu9250_imu_gyro_radps[i] =
      ref.sensor.mpu9250_imu.gyro_radps[i];
    datalog_msg_.mpu9250_imu_mag_ut[i] = ref.sensor.mpu9250_imu.mag_ut[i];
  }
  /* VectorNav imu */
  datalog_msg_.vector_nav_imu_installed = ref.sensor.vector_nav_imu.installed;
  datalog_msg_.vector_nav_imu_healthy = ref.sensor.vector_nav_imu.healthy;
  datalog_msg_.vector_nav_imu_new_imu_data =
    ref.sensor.vector_nav_imu.new_imu_data;
  datalog_msg_.vector_nav_imu_new_mag_data =
    ref.sensor.vector_nav_imu.new_mag_data;
  datalog_msg_.vector_nav_imu_die_temp_c = ref.sensor.vector_nav_imu.die_temp_c;
  for (std::size_t i = 0; i < 3; i++) {
    datalog_msg_.vector_nav_imu_accel_mps2[i] =
      ref.sensor.vector_nav_imu.accel_mps2[i];
    datalog_msg_.vector_nav_imu_gyro_radps[i] =
      ref.sensor.vector_nav_imu.gyro_radps[i];
    datalog_msg_.vector_nav_imu_mag_ut[i] = ref.sensor.vector_nav_imu.mag_ut[i];
  }
  /* BME280 static pressure */
  datalog_msg_.bme280_static_pres_installed =
    ref.sensor.bme280_static_pres.installed;
  datalog_msg_.bme280_static_pres_healthy =
    ref.sensor.bme280_static_pres.healthy;
  datalog_msg_.bme280_static_pres_new_data =
    ref.sensor.bme280_static_pres.new_data;
  datalog_msg_.bme280_static_pres_die_temp_c =
    ref.sensor.bme280_static_pres.die_temp_c;
  datalog_msg_.bme280_static_pres_pa = ref.sensor.bme280_static_pres.pres_pa;
  /* VectorNav static pressure */
  datalog_msg_.vector_nav_static_pres_installed =
    ref.sensor.vector_nav_static_pres.installed;
  datalog_msg_.vector_nav_static_pres_healthy =
    ref.sensor.vector_nav_static_pres.healthy;
  datalog_msg_.vector_nav_static_pres_new_data =
    ref.sensor.vector_nav_static_pres.new_data;
  datalog_msg_.vector_nav_static_pres_die_temp_c =
    ref.sensor.vector_nav_static_pres.die_temp_c;
  datalog_msg_.vector_nav_static_pres_pa =
    ref.sensor.vector_nav_static_pres.pres_pa;
  /* AMS5915 static pressure */
  datalog_msg_.ams5915_static_pres_installed =
    ref.sensor.ams5915_static_pres.installed;
  datalog_msg_.ams5915_static_pres_healthy =
    ref.sensor.ams5915_static_pres.healthy;
  datalog_msg_.ams5915_static_pres_new_data =
    ref.sensor.ams5915_static_pres.new_data;
  datalog_msg_.ams5915_static_pres_die_temp_c =
    ref.sensor.ams5915_static_pres.die_temp_c;
  datalog_msg_.ams5915_static_pres_pa =
    ref.sensor.ams5915_static_pres.pres_pa;
  /* AMS5915 differential pressure */
  datalog_msg_.ams5915_diff_pres_installed =
    ref.sensor.ams5915_diff_pres.installed;
  datalog_msg_.ams5915_diff_pres_healthy =
    ref.sensor.ams5915_diff_pres.healthy;
  datalog_msg_.ams5915_diff_pres_new_data =
    ref.sensor.ams5915_diff_pres.new_data;
  datalog_msg_.ams5915_diff_pres_die_temp_c =
    ref.sensor.ams5915_diff_pres.die_temp_c;
  datalog_msg_.ams5915_diff_pres_pa =
    ref.sensor.ams5915_diff_pres.pres_pa;
  /* VectorNav GNSS data */
  datalog_msg_.vector_nav_gnss_installed = ref.sensor.vector_nav_gnss.installed;
  datalog_msg_.vector_nav_gnss_healthy = ref.sensor.vector_nav_gnss.healthy;
  datalog_msg_.vector_nav_gnss_new_data = ref.sensor.vector_nav_gnss.new_data;
  datalog_msg_.vector_nav_gnss_fix = ref.sensor.vector_nav_gnss.fix;
  datalog_msg_.vector_nav_gnss_num_sats = ref.sensor.vector_nav_gnss.num_sats;
  datalog_msg_.vector_nav_gnss_week = ref.sensor.vector_nav_gnss.gps_week;
  datalog_msg_.vector_nav_gnss_alt_wgs84_m =
    ref.sensor.vector_nav_gnss.alt_wgs84_m;
  datalog_msg_.vector_nav_gnss_horz_acc_m =
    ref.sensor.vector_nav_gnss.horz_acc_m;
  datalog_msg_.vector_nav_gnss_vert_acc_m =
    ref.sensor.vector_nav_gnss.vert_acc_m;
  datalog_msg_.vector_nav_gnss_vel_acc_mps =
    ref.sensor.vector_nav_gnss.vel_acc_mps;
  for (std::size_t i = 0; i < 3; i++) {
    datalog_msg_.vector_nav_gnss_ned_vel_mps[i] =
      ref.sensor.vector_nav_gnss.ned_vel_mps[i];
  }
  datalog_msg_.vector_nav_gnss_tow_s = ref.sensor.vector_nav_gnss.gps_tow_s;
  datalog_msg_.vector_nav_gnss_lat_rad = ref.sensor.vector_nav_gnss.lat_rad;
  datalog_msg_.vector_nav_gnss_lon_rad = ref.sensor.vector_nav_gnss.lon_rad;
  /* uBlox3 GNSS data */
  datalog_msg_.ublox3_gnss_installed = ref.sensor.ublox3_gnss.installed;
  datalog_msg_.ublox3_gnss_healthy = ref.sensor.ublox3_gnss.healthy;
  datalog_msg_.ublox3_gnss_new_data = ref.sensor.ublox3_gnss.new_data;
  datalog_msg_.ublox3_gnss_fix = ref.sensor.ublox3_gnss.fix;
  datalog_msg_.ublox3_gnss_num_sats = ref.sensor.ublox3_gnss.num_sats;
  datalog_msg_.ublox3_gnss_week = ref.sensor.ublox3_gnss.gps_week;
  datalog_msg_.ublox3_gnss_alt_wgs84_m = ref.sensor.ublox3_gnss.alt_wgs84_m;
  datalog_msg_.ublox3_gnss_horz_acc_m = ref.sensor.ublox3_gnss.horz_acc_m;
  datalog_msg_.ublox3_gnss_vert_acc_m = ref.sensor.ublox3_gnss.vert_acc_m;
  datalog_msg_.ublox3_gnss_vel_acc_mps = ref.sensor.ublox3_gnss.vel_acc_mps;
  for (std::size_t i = 0; i < 3; i++) {
    datalog_msg_.ublox3_gnss_ned_vel_mps[i] =
      ref.sensor.ublox3_gnss.ned_vel_mps[i];
  }
  datalog_msg_.ublox3_gnss_tow_s = ref.sensor.ublox3_gnss.gps_tow_s;
  datalog_msg_.ublox3_gnss_lat_rad = ref.sensor.ublox3_gnss.lat_rad;
  datalog_msg_.ublox3_gnss_lon_rad = ref.sensor.ublox3_gnss.lon_rad;
  /* uBlox4 GNSS data */
  datalog_msg_.ublox4_gnss_installed = ref.sensor.ublox4_gnss.installed;
  datalog_msg_.ublox4_gnss_healthy = ref.sensor.ublox4_gnss.healthy;
  datalog_msg_.ublox4_gnss_new_data = ref.sensor.ublox4_gnss.new_data;
  datalog_msg_.ublox4_gnss_fix = ref.sensor.ublox4_gnss.fix;
  datalog_msg_.ublox4_gnss_num_sats = ref.sensor.ublox4_gnss.num_sats;
  datalog_msg_.ublox4_gnss_week = ref.sensor.ublox4_gnss.gps_week;
  datalog_msg_.ublox4_gnss_alt_wgs84_m = ref.sensor.ublox4_gnss.alt_wgs84_m;
  datalog_msg_.ublox4_gnss_horz_acc_m = ref.sensor.ublox4_gnss.horz_acc_m;
  datalog_msg_.ublox4_gnss_vert_acc_m = ref.sensor.ublox4_gnss.vert_acc_m;
  datalog_msg_.ublox4_gnss_vel_acc_mps = ref.sensor.ublox4_gnss.vel_acc_mps;
  for (std::size_t i = 0; i < 3; i++) {
    datalog_msg_.ublox4_gnss_ned_vel_mps[i] =
      ref.sensor.ublox4_gnss.ned_vel_mps[i];
  }
  datalog_msg_.ublox4_gnss_tow_s = ref.sensor.ublox4_gnss.gps_tow_s;
  datalog_msg_.ublox4_gnss_lat_rad = ref.sensor.ublox4_gnss.lat_rad;
  datalog_msg_.ublox4_gnss_lon_rad = ref.sensor.ublox4_gnss.lon_rad;
  /* uBlox3 rel pos data */
  datalog_msg_.ublox3_relpos_avail =
    ref.sensor.ublox3_relpos.avail;
  datalog_msg_.ublox3_relpos_moving_baseline =
    ref.sensor.ublox3_relpos.moving_baseline;
  datalog_msg_.ublox3_relpos_heading_valid =
    ref.sensor.ublox3_relpos.heading_valid;
  datalog_msg_.ublox3_relpos_normalized =
    ref.sensor.ublox3_relpos.baseline_normalized;
  datalog_msg_.ublox3_relpos_len_acc_m =
    ref.sensor.ublox3_relpos.baseline_len_acc_m;
  datalog_msg_.ublox3_relpos_heading_rad = ref.sensor.ublox3_relpos.heading_rad;
  datalog_msg_.ublox3_relpos_heading_acc_rad =
    ref.sensor.ublox3_relpos.heading_acc_rad;
  datalog_msg_.ublox3_relpos_len_m = ref.sensor.ublox3_relpos.baseline_len_m;
  for (std::size_t i = 0; i < 3; i++) {
    datalog_msg_.ublox3_relpos_acc_ned_m[i] =
      ref.sensor.ublox3_relpos.rel_pos_acc_ned_m[i];
    datalog_msg_.ublox3_relpos_ned_m[i] =
      ref.sensor.ublox3_relpos.rel_pos_ned_m[i];
  }
  /* uBlox4 rel pos data */
  datalog_msg_.ublox4_relpos_avail = ref.sensor.ublox4_relpos.avail;
  datalog_msg_.ublox4_relpos_moving_baseline =
    ref.sensor.ublox4_relpos.moving_baseline;
  datalog_msg_.ublox4_relpos_heading_valid =
    ref.sensor.ublox4_relpos.heading_valid;
  datalog_msg_.ublox4_relpos_normalized =
    ref.sensor.ublox4_relpos.baseline_normalized;
  datalog_msg_.ublox4_relpos_len_acc_m =
    ref.sensor.ublox4_relpos.baseline_len_acc_m;
  datalog_msg_.ublox4_relpos_heading_rad = ref.sensor.ublox4_relpos.heading_rad;
  datalog_msg_.ublox4_relpos_heading_acc_rad =
    ref.sensor.ublox4_relpos.heading_acc_rad;
  datalog_msg_.ublox4_relpos_len_m = ref.sensor.ublox4_relpos.baseline_len_m;
  for (std::size_t i = 0; i < 3; i++) {
    datalog_msg_.ublox4_relpos_acc_ned_m[i] =
      ref.sensor.ublox4_relpos.rel_pos_acc_ned_m[i];
    datalog_msg_.ublox4_relpos_ned_m[i] =
      ref.sensor.ublox4_relpos.rel_pos_ned_m[i];
  }
  /* ADC */
  for (std::size_t i = 0; i < NUM_AIN_PINS; i++) {
    datalog_msg_.adc_volt[i] = ref.sensor.adc.volt[i];
  }
  /* Power Module */
  #if defined(__FMU_R_V2__)
  datalog_msg_.pwr_mod_volt_v = ref.sensor.power_module.voltage_v;
  datalog_msg_.pwr_mod_curr_v = ref.sensor.power_module.current_v;
  #endif
  /* Air data */
  datalog_msg_.airdata_initialized = ref.nav.airdata.initialized;
  datalog_msg_.airdata_static_pres_pa = ref.nav.airdata.static_pres_pa;
  datalog_msg_.airdata_diff_pres_pa = ref.nav.airdata.diff_pres_pa;
  datalog_msg_.airdata_alt_pres_m = ref.nav.airdata.alt_pres_m;
  datalog_msg_.airdata_ias_mps = ref.nav.airdata.ias_mps;
  /* BFS EKF */
  datalog_msg_.bfs_nav_healthy = ref.nav.bfs_ekf.healthy;
  datalog_msg_.bfs_nav_pitch_rad = ref.nav.bfs_ekf.pitch_rad;
  datalog_msg_.bfs_nav_roll_rad = ref.nav.bfs_ekf.roll_rad;
  datalog_msg_.bfs_nav_heading_rad = ref.nav.bfs_ekf.heading_rad;
  datalog_msg_.bfs_nav_alt_wgs84_m = ref.nav.bfs_ekf.alt_wgs84_m;
  for (std::size_t i = 0; i < 3; i++) {
    datalog_msg_.bfs_nav_accel_mps2[i] = ref.nav.bfs_ekf.accel_mps2[i];
    datalog_msg_.bfs_nav_gyro_radps[i] = ref.nav.bfs_ekf.gyro_radps[i];
    datalog_msg_.bfs_nav_mag_ut[i] = ref.nav.bfs_ekf.mag_ut[i];
    datalog_msg_.bfs_nav_ned_vel_mps[i] = ref.nav.bfs_ekf.ned_vel_mps[i];
  }
  datalog_msg_.bfs_nav_lat_rad = ref.nav.bfs_ekf.lat_rad;
  datalog_msg_.bfs_nav_lon_rad = ref.nav.bfs_ekf.lon_rad;
  /* VectorNav EKF */
  datalog_msg_.vector_nav_healthy = ref.nav.vector_nav.healthy;
  datalog_msg_.vector_nav_pitch_rad = ref.nav.vector_nav.pitch_rad;
  datalog_msg_.vector_nav_roll_rad = ref.nav.vector_nav.roll_rad;
  datalog_msg_.vector_nav_heading_rad = ref.nav.vector_nav.heading_rad;
  datalog_msg_.vector_nav_alt_wgs84_m = ref.nav.vector_nav.alt_wgs84_m;
  for (std::size_t i = 0; i < 3; i++) {
    datalog_msg_.vector_nav_accel_mps2[i] = ref.nav.vector_nav.accel_mps2[i];
    datalog_msg_.vector_nav_gyro_radps[i] = ref.nav.vector_nav.gyro_radps[i];
    datalog_msg_.vector_nav_mag_ut[i] = ref.nav.vector_nav.mag_ut[i];
    datalog_msg_.vector_nav_ned_vel_mps[i] = ref.nav.vector_nav.ned_vel_mps[i];
  }
  datalog_msg_.vector_nav_lat_rad = ref.nav.vector_nav.lat_rad;
  datalog_msg_.vector_nav_lon_rad = ref.nav.vector_nav.lon_rad;
  /* VMS data */
  datalog_msg_.vms_waypoint_reached = ref.vms.waypoint_reached;
  datalog_msg_.vms_sbus_ch17 = ref.vms.sbus.ch17;
  datalog_msg_.vms_sbus_ch18 = ref.vms.sbus.ch18;
  datalog_msg_.vms_motors_enabled = ref.vms.motors_enabled;
  datalog_msg_.vms_mode = ref.vms.mode;
  datalog_msg_.vms_throttle_cmd_prcnt = ref.vms.throttle_cmd_prcnt;
  for (std::size_t i = 0; i < NUM_SBUS_CH; i++) {
    datalog_msg_.vms_sbus_cnt[i] = ref.vms.sbus.cnt[i];
    datalog_msg_.vms_sbus_cmd[i] = ref.vms.sbus.cmd[i];
  }
  for (std::size_t i = 0; i < NUM_PWM_PINS; i++) {
    datalog_msg_.vms_pwm_cnt[i] = ref.vms.pwm.cnt[i];
    datalog_msg_.vms_pwm_cmd[i] = ref.vms.pwm.cmd[i];
  }
  for (std::size_t i = 0; i < NUM_AUX_VAR; i++) {
    datalog_msg_.vms_aux[i] = ref.vms.aux[i];
  }
  for (std::size_t i = 0; i < NUM_AIN_PINS; i++) {
    datalog_msg_.vms_analog[i] = ref.vms.analog.val[i];
  }
  #if defined(__FMU_R_V2__)
  datalog_msg_.vms_batt_volt_v = ref.vms.battery.voltage_v;
  datalog_msg_.vms_batt_curr_ma = ref.vms.battery.current_ma;
  datalog_msg_.vms_batt_consumed_mah = ref.vms.battery.consumed_mah;
  datalog_msg_.vms_batt_remaining_prcnt = ref.vms.battery.remaining_prcnt;
  datalog_msg_.vms_batt_remaining_time_s = ref.vms.battery.remaining_time_s;
  #endif
  /* Telemetry data */
  for (std::size_t i = 0; i < NUM_TELEM_PARAMS; i++) {
    datalog_msg_.telem_param[i] = ref.telem.param[i];
  }
  datalog_msg_.waypoint_frame =
    ref.telem.flight_plan[ref.telem.current_waypoint].frame;
  datalog_msg_.waypoint_cmd =
    ref.telem.flight_plan[ref.telem.current_waypoint].cmd;
  datalog_msg_.waypoint_param1 =
    ref.telem.flight_plan[ref.telem.current_waypoint].param1;
  datalog_msg_.waypoint_param2 =
    ref.telem.flight_plan[ref.telem.current_waypoint].param2;
  datalog_msg_.waypoint_param3 =
    ref.telem.flight_plan[ref.telem.current_waypoint].param3;
  datalog_msg_.waypoint_param4 =
    ref.telem.flight_plan[ref.telem.current_waypoint].param4;
  datalog_msg_.waypoint_x =
    ref.telem.flight_plan[ref.telem.current_waypoint].x;
  datalog_msg_.waypoint_y =
    ref.telem.flight_plan[ref.telem.current_waypoint].y;
  datalog_msg_.waypoint_z =
    ref.telem.flight_plan[ref.telem.current_waypoint].z;

  /* Encode */
  stream_ = pb_ostream_from_buffer(data_buffer_, sizeof(data_buffer_));
  if (!pb_encode(&stream_, DatalogMessage_fields, &datalog_msg_)) {
    MsgWarning("Error encoding datalog.");
    return;
  }
  std::size_t msg_len = stream_.bytes_written;
  /* Framing */
  std::size_t bytes_written = encoder.Write(data_buffer_, msg_len);
  if (msg_len != bytes_written) {
    MsgWarning("Error framing datalog.");
    return;
  }
  /* Write the data */
  logger_.Write(const_cast<uint8_t *>(encoder.data()), encoder.size());
}
void DatalogClose() {
  logger_.Close();
}
void DatalogFlush() {
  logger_.Flush();
}
