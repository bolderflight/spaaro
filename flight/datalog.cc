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

#include "flight/hardware_defs.h"
#include "flight/global_defs.h"
#include "flight/config.h"
#include "flight/msg.h"
#include "flight/datalog.h"
#include "logger/logger.h"
#include "framing/framing.h"
#include "datalog.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

namespace {
/* Datalog file name */
std::string DATA_LOG_NAME_ = "flight_data";
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
  datalog_msg_.sys_frame_time_us = ref.sys_mon.frame_time_us;
  datalog_msg_.sys_input_volt = ref.sys_mon.input_volt;
  datalog_msg_.sys_reg_volt = ref.sys_mon.reg_volt;
  datalog_msg_.sys_pwm_volt = ref.sys_mon.pwm_volt;
  datalog_msg_.sys_sbus_volt = ref.sys_mon.sbus_volt;
  datalog_msg_.sys_time_s = ref.sys_mon.sys_time_s;
  datalog_msg_.inceptor_new_data = ref.inceptor.new_data;
  datalog_msg_.inceptor_mode0 = ref.inceptor.mode0;
  datalog_msg_.inceptor_mode1 = ref.inceptor.mode1;
  datalog_msg_.inceptor_throttle_en = ref.inceptor.throttle_en;
  datalog_msg_.inceptor_lost_frame = ref.inceptor.lost_frame;
  datalog_msg_.inceptor_failsafe = ref.inceptor.failsafe;
  datalog_msg_.inceptor_throttle = ref.inceptor.throttle;
  datalog_msg_.inceptor_pitch = ref.inceptor.pitch;
  datalog_msg_.inceptor_roll = ref.inceptor.roll;
  datalog_msg_.inceptor_yaw = ref.inceptor.yaw;
  datalog_msg_.imu_new_data = ref.sensor.imu.new_data;
  datalog_msg_.imu_accel_x_mps2 = ref.sensor.imu.accel_x_mps2;
  datalog_msg_.imu_accel_y_mps2 = ref.sensor.imu.accel_y_mps2;
  datalog_msg_.imu_accel_z_mps2 = ref.sensor.imu.accel_z_mps2;
  datalog_msg_.imu_gyro_x_radps = ref.sensor.imu.gyro_x_radps;
  datalog_msg_.imu_gyro_y_radps = ref.sensor.imu.gyro_y_radps;
  datalog_msg_.imu_gyro_z_radps = ref.sensor.imu.gyro_z_radps;
  datalog_msg_.imu_mag_x_ut = ref.sensor.imu.mag_x_ut;
  datalog_msg_.imu_mag_y_ut = ref.sensor.imu.mag_y_ut;
  datalog_msg_.imu_mag_z_ut = ref.sensor.imu.mag_z_ut;
  datalog_msg_.imu_dlpf_accel_x_mps2 = ref.sensor.imu.dlpf.accel_x_mps2;
  datalog_msg_.imu_dlpf_accel_y_mps2 = ref.sensor.imu.dlpf.accel_y_mps2;
  datalog_msg_.imu_dlpf_accel_z_mps2 = ref.sensor.imu.dlpf.accel_z_mps2;
  datalog_msg_.imu_dlpf_gyro_x_radps = ref.sensor.imu.dlpf.gyro_x_radps;
  datalog_msg_.imu_dlpf_gyro_y_radps = ref.sensor.imu.dlpf.gyro_y_radps;
  datalog_msg_.imu_dlpf_gyro_z_radps = ref.sensor.imu.dlpf.gyro_z_radps;
  datalog_msg_.imu_dlpf_mag_x_ut = ref.sensor.imu.dlpf.mag_x_ut;
  datalog_msg_.imu_dlpf_mag_y_ut = ref.sensor.imu.dlpf.mag_y_ut;
  datalog_msg_.imu_dlpf_mag_z_ut = ref.sensor.imu.dlpf.mag_z_ut;
  datalog_msg_.imu_die_temp_c = ref.sensor.imu.die_temp_c;
  datalog_msg_.gnss_new_data = ref.sensor.gnss.new_data;
  datalog_msg_.gnss_fix = ref.sensor.gnss.fix;
  datalog_msg_.gnss_num_sats = ref.sensor.gnss.num_sats;
  datalog_msg_.gnss_week = ref.sensor.gnss.week;
  datalog_msg_.gnss_alt_wgs84_m = ref.sensor.gnss.alt_wgs84_m;
  datalog_msg_.gnss_north_vel_mps = ref.sensor.gnss.north_vel_mps;
  datalog_msg_.gnss_east_vel_mps = ref.sensor.gnss.east_vel_mps;
  datalog_msg_.gnss_down_vel_mps = ref.sensor.gnss.down_vel_mps;
  datalog_msg_.gnss_horz_acc_m = ref.sensor.gnss.horz_acc_m;
  datalog_msg_.gnss_vert_acc_m = ref.sensor.gnss.vert_acc_m;
  datalog_msg_.gnss_vel_acc_mps = ref.sensor.gnss.vel_acc_mps;
  datalog_msg_.gnss_lat_rad = ref.sensor.gnss.lat_rad;
  datalog_msg_.gnss_lon_rad = ref.sensor.gnss.lon_rad;
  datalog_msg_.gnss_tow_s = ref.sensor.gnss.tow_s;
  datalog_msg_.airdata_diff_new_data = ref.sensor.airdata.diff_pres.new_data;
  datalog_msg_.airdata_diff_pres_pa = ref.sensor.airdata.diff_pres.pres_pa;
  datalog_msg_.airdata_dlpf_diff_pres_pa = ref.sensor.airdata.diff_pres.dlpf.pres_pa;
  datalog_msg_.airdata_diff_die_temp_c = ref.sensor.airdata.diff_pres.die_temp_c;
  datalog_msg_.airdata_static_new_data = ref.sensor.airdata.static_pres.new_data;
  datalog_msg_.airdata_static_pres_pa = ref.sensor.airdata.static_pres.pres_pa;
  datalog_msg_.airdata_dlpf_static_pres_pa = ref.sensor.airdata.static_pres.dlpf.pres_pa;
  datalog_msg_.airdata_static_die_temp_c = ref.sensor.airdata.static_pres.die_temp_c;
  datalog_msg_.nav_accel_x_mps2 = ref.nav.accel_x_mps2;
  datalog_msg_.nav_accel_y_mps2 = ref.nav.accel_y_mps2;
  datalog_msg_.nav_accel_z_mps2 = ref.nav.accel_z_mps2;
  datalog_msg_.nav_gyro_x_radps = ref.nav.gyro_x_radps;
  datalog_msg_.nav_gyro_y_radps = ref.nav.gyro_y_radps;
  datalog_msg_.nav_gyro_z_radps = ref.nav.gyro_z_radps;
  datalog_msg_.nav_pitch_rad = ref.nav.pitch_rad;
  datalog_msg_.nav_roll_rad = ref.nav.roll_rad;
  datalog_msg_.nav_heading_rad = ref.nav.heading_rad;
  datalog_msg_.nav_alt_wgs84_m = ref.nav.alt_wgs84_m;
  datalog_msg_.nav_alt_pres_m = ref.nav.alt_pres_m;
  datalog_msg_.nav_ias_mps = ref.nav.ias_mps;
  datalog_msg_.nav_eas_mps = ref.nav.eas_mps;
  datalog_msg_.nav_gnd_spd_mps = ref.nav.gnd_spd_mps;
  datalog_msg_.nav_gnd_track_rad = ref.nav.gnd_track_rad;
  datalog_msg_.nav_flight_path_rad = ref.nav.flight_path_rad;
  datalog_msg_.nav_north_vel_mps = ref.nav.north_vel_mps;
  datalog_msg_.nav_east_vel_mps = ref.nav.east_vel_mps;
  datalog_msg_.nav_down_vel_mps = ref.nav.down_vel_mps;
  datalog_msg_.nav_lat_rad = ref.nav.lat_rad;
  datalog_msg_.nav_lon_rad = ref.nav.lon_rad;
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
