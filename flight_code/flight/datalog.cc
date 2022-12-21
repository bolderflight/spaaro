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

#include "flight/datalog.h"
#include "flight/msg.h"
#include "logger/logger.h"
#include "framing/framing.h"
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
static const char * DATA_LOG_NAME_ = "ale";
/* SD card */
SdFat32 sd_;
/* Logger object */
bfs::Logger<400> logger_(&sd_);
/* Framing */
bfs::Encoder<DatalogMessage_size> encoder;
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
  /* System data */
  datalog_msg_.sys_frame_time_us = ref.sys.frame_time_us;
  #if defined(__FMU_R_V1__)
  datalog_msg_.sys_input_volt = ref.sys.input_volt;
  datalog_msg_.sys_reg_volt = ref.sys.reg_volt;
  datalog_msg_.sys_pwm_volt = ref.sys.pwm_volt;
  datalog_msg_.sys_sbus_volt = ref.sys.sbus_volt;
  #endif
  datalog_msg_.sys_time_s = static_cast<double>(ref.sys.sys_time_us) / 1e6;
  /* Inceptor data */
  datalog_msg_.incept_new_data = ref.sensor.inceptor.new_data;
  datalog_msg_.incept_lost_frame = ref.sensor.inceptor.lost_frame;
  datalog_msg_.incept_failsafe = ref.sensor.inceptor.failsafe;
  datalog_msg_.incept_ch17 = ref.sensor.inceptor.ch17;
  datalog_msg_.incept_ch18 = ref.sensor.inceptor.ch18;
  for (std::size_t i = 0; i < NUM_SBUS_CH; i++) {
    datalog_msg_.incept_ch[i] = ref.sensor.inceptor.ch[i];
  }
  /* IMU data */
  datalog_msg_.imu_new_data = ref.sensor.imu.new_imu_data;
  datalog_msg_.imu_new_mag_data = ref.sensor.imu.new_mag_data;
  datalog_msg_.imu_healthy = ref.sensor.imu.imu_healthy;
  datalog_msg_.imu_mag_healthy = ref.sensor.imu.mag_healthy;
  datalog_msg_.imu_die_temp_c = ref.sensor.imu.die_temp_c;
  for (std::size_t i = 0; i < 3; i++) {
    datalog_msg_.imu_accel_mps2[i] = ref.sensor.imu.accel_mps2[i];
    datalog_msg_.imu_gyro_radps[i] = ref.sensor.imu.gyro_radps[i];
    datalog_msg_.imu_mag_ut[i] = ref.sensor.imu.mag_ut[i];
  }
  /* GNSS data */
  datalog_msg_.gnss_new_data = ref.sensor.gnss.new_data;
  datalog_msg_.gnss_healthy = ref.sensor.gnss.healthy;
  datalog_msg_.gnss_fix = ref.sensor.gnss.fix;
  datalog_msg_.gnss_num_sats = ref.sensor.gnss.num_sats;
  datalog_msg_.gnss_week = ref.sensor.gnss.week;
  datalog_msg_.gnss_tow_ms = ref.sensor.gnss.tow_ms;
  datalog_msg_.gnss_alt_wgs84_m = ref.sensor.gnss.alt_wgs84_m;
  datalog_msg_.gnss_alt_msl_m = ref.sensor.gnss.alt_msl_m;
  datalog_msg_.gnss_hdop = ref.sensor.gnss.hdop;
  datalog_msg_.gnss_vdop = ref.sensor.gnss.vdop;
  datalog_msg_.gnss_track_rad = ref.sensor.gnss.track_rad;
  datalog_msg_.gnss_spd_mps = ref.sensor.gnss.spd_mps;
  datalog_msg_.gnss_horz_acc_m = ref.sensor.gnss.horz_acc_m;
  datalog_msg_.gnss_vert_acc_m = ref.sensor.gnss.vert_acc_m;
  datalog_msg_.gnss_vel_acc_mps = ref.sensor.gnss.vel_acc_mps;
  datalog_msg_.gnss_track_acc_rad = ref.sensor.gnss.track_acc_rad;
  for (std::size_t i = 0; i < 3; i++) {
    datalog_msg_.gnss_ned_vel_mps[i] = ref.sensor.gnss.ned_vel_mps[i];
  }
  datalog_msg_.gnss_lat_rad = ref.sensor.gnss.lat_rad;
  datalog_msg_.gnss_lon_rad = ref.sensor.gnss.lon_rad;
  /* Pressure data */
  datalog_msg_.pitot_static_installed = ref.sensor.pitot_static_installed;
  datalog_msg_.pres_static_new_data = ref.sensor.static_pres.new_data;
  datalog_msg_.pres_static_healthy = ref.sensor.static_pres.healthy;
  datalog_msg_.pres_static_pres_pa = ref.sensor.static_pres.pres_pa;
  datalog_msg_.pres_static_die_temp_c = ref.sensor.static_pres.die_temp_c;
  datalog_msg_.pres_diff_new_data = ref.sensor.diff_pres.new_data;
  datalog_msg_.pres_diff_healthy = ref.sensor.diff_pres.healthy;
  datalog_msg_.pres_diff_pres_pa = ref.sensor.diff_pres.pres_pa;
  datalog_msg_.pres_diff_die_temp_c = ref.sensor.diff_pres.die_temp_c;
  /* Analog data */
  for (std::size_t i = 0; i < NUM_AIN_PINS; i++) {
    datalog_msg_.adc_volt[i] = ref.sensor.adc.volt[i];
  }
  /* Power module data */
  #if defined(__FMU_R_V2__)
  datalog_msg_.pwr_mod_volt_v = ref.sensor.power_module.voltage_v;
  datalog_msg_.pwr_mod_curr_v = ref.sensor.power_module.current_v;
  #endif
  /* Nav data */
  datalog_msg_.nav_initialized = ref.nav.nav_initialized;
  datalog_msg_.nav_pitch_rad = ref.nav.pitch_rad;
  datalog_msg_.nav_roll_rad = ref.nav.roll_rad;
  datalog_msg_.nav_heading_rad = ref.nav.heading_rad;
  datalog_msg_.nav_alt_wgs84_m = ref.nav.alt_wgs84_m;
  datalog_msg_.nav_home_alt_wgs84_m = ref.nav.home_alt_wgs84_m;
  datalog_msg_.nav_alt_msl_m = ref.nav.alt_msl_m;
  datalog_msg_.nav_alt_rel_m = ref.nav.alt_rel_m;
  datalog_msg_.nav_static_pres_pa = ref.nav.static_pres_pa;
  datalog_msg_.nav_diff_pres_pa = ref.nav.diff_pres_pa;
  datalog_msg_.nav_alt_pres_m = ref.nav.alt_pres_m;
  datalog_msg_.nav_ias_mps = ref.nav.ias_mps;
  datalog_msg_.nav_gnd_spd_mps = ref.nav.gnd_spd_mps;
  datalog_msg_.nav_gnd_track_rad = ref.nav.gnd_track_rad;
  datalog_msg_.nav_flight_path_rad = ref.nav.flight_path_rad;
  for (std::size_t i = 0; i < 3; i++) {
    datalog_msg_.nav_accel_bias_mps2[i] = ref.nav.accel_bias_mps2[i];
    datalog_msg_.nav_gyro_bias_radps[i] = ref.nav.gyro_bias_radps[i];
    datalog_msg_.nav_accel_mps2[i] = ref.nav.accel_mps2[i];
    datalog_msg_.nav_gyro_radps[i] = ref.nav.gyro_radps[i];
    datalog_msg_.nav_mag_ut[i] = ref.nav.mag_ut[i];
    datalog_msg_.nav_ned_pos_m[i] = ref.nav.ned_pos_m[i];
    datalog_msg_.nav_ned_vel_mps[i] = ref.nav.ned_vel_mps[i];
  }
  datalog_msg_.nav_lat_rad = ref.nav.lat_rad;
  datalog_msg_.nav_lon_rad = ref.nav.lon_rad;
  datalog_msg_.nav_home_lat_rad = ref.nav.home_lat_rad;
  datalog_msg_.nav_home_lon_rad = ref.nav.home_lon_rad;
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
  logger_.Write(encoder.Data(), encoder.Size());
}
void DatalogClose() {
  logger_.Close();
}
void DatalogFlush() {
  logger_.Flush();
}
