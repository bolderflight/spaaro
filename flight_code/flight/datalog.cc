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
#include "framing.h"
#include "../../common/datalog_fmu.h"
#include "units.h"
#include "flight/telem.h"

namespace {
/* Datalog file name */
static const char * DATA_LOG_NAME_ = "malt1_";
/* SD card */
SdFat32 sd_;
/* Logger object */
bfs::Logger<400> logger_(&sd_);
/* Datalog message */
DatalogMsg datalog_msg_;
/* Framing */
bfs::FrameEncoder<sizeof(datalog_msg_)> encoder;
/* Temp */
uint32_t temp_;
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
  MsgInfo("done. Log file prefix is ");
  MsgInfo(DATA_LOG_NAME_);
  MsgInfo("xx.bfs\n");
}

template<typename T>
uint64_t Scale(T val, T min, T max, T sf, T bias) {
  static_assert(std::is_floating_point<T>::value,
                "Only floating point types supported");
  if (val < min) {val = min;}
  if (val > max) {val = max;}
  return static_cast<uint64_t>(val * sf + bias);
}

void DatalogAdd(const AircraftData &ref) {
  /* Assign to message */

  /* SysData */
  datalog_msg_.sys_frame_time_us = ref.sys.frame_time_us;
  #if defined(__FMU_R_V1__)
  datalog_msg_.sys_input_volt = Scale(ref.sys.input_volt, 0.0f, 36.0f, 113.75f, 0.0f);
  datalog_msg_.sys_reg_volt = Scale(ref.sys.reg_volt, 2.5f, 6.5f, 127.75f, -319.375f);
  datalog_msg_.sys_pwm_volt = Scale(ref.sys.pwm_volt, 0.0f, 10.0f, 102.3f, 0.0f);
  datalog_msg_.sys_sbus_volt = Scale(ref.sys.sbus_volt, 0.0f, 10.0f, 102.3f, 0.0f);
  #endif
  datalog_msg_.sys_time_us = ref.sys.sys_time_us;

  /* Inceptor */
  datalog_msg_.incept_new_data = ref.sensor.inceptor.new_data;
  datalog_msg_.incept_lost_frame = ref.sensor.inceptor.lost_frame;
  datalog_msg_.incept_failsafe = ref.sensor.inceptor.failsafe;
  datalog_msg_.incept_ch0 = ref.sensor.inceptor.ch[0];
  datalog_msg_.incept_ch1 = ref.sensor.inceptor.ch[1];
  datalog_msg_.incept_ch2 = ref.sensor.inceptor.ch[2];
  datalog_msg_.incept_ch3 = ref.sensor.inceptor.ch[3];
  datalog_msg_.incept_ch4 = ref.sensor.inceptor.ch[4];
  datalog_msg_.incept_ch5 = ref.sensor.inceptor.ch[5];
  datalog_msg_.incept_ch6 = ref.sensor.inceptor.ch[6];
  datalog_msg_.incept_ch7 = ref.sensor.inceptor.ch[7];
  datalog_msg_.incept_ch8 = ref.sensor.inceptor.ch[8];
  datalog_msg_.incept_ch9 = ref.sensor.inceptor.ch[9];
  datalog_msg_.incept_ch10 = ref.sensor.inceptor.ch[10];
  datalog_msg_.incept_ch11 = ref.sensor.inceptor.ch[11];
  datalog_msg_.incept_ch12 = ref.sensor.inceptor.ch[12];
  datalog_msg_.incept_ch13 = ref.sensor.inceptor.ch[13];
  datalog_msg_.incept_ch14 = ref.sensor.inceptor.ch[14];
  datalog_msg_.incept_ch15 = ref.sensor.inceptor.ch[15];

  /* FMU IMU */
  datalog_msg_.fmu_imu_healthy = ref.sensor.fmu_imu.healthy;
  datalog_msg_.fmu_imu_new_data = ref.sensor.fmu_imu.new_data;
  datalog_msg_.fmu_imu_die_temp_c = Scale(ref.sensor.fmu_imu.die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f);
  datalog_msg_.fmu_imu_accel_x_mps2 = Scale(ref.sensor.fmu_imu.accel_mps2[0], -156.9064f, 156.9064f, 208.834693804714f, 32767.5f);
  datalog_msg_.fmu_imu_accel_y_mps2 = Scale(ref.sensor.fmu_imu.accel_mps2[1], -156.9064f, 156.9064f, 208.834693804714f, 32767.5f);
  datalog_msg_.fmu_imu_accel_z_mps2 = Scale(ref.sensor.fmu_imu.accel_mps2[2], -156.9064f, 156.9064f, 208.834693804714f, 32767.5f);
  datalog_msg_.fmu_imu_gyro_x_radps = Scale(ref.sensor.fmu_imu.gyro_radps[0], -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f);
  datalog_msg_.fmu_imu_gyro_y_radps = Scale(ref.sensor.fmu_imu.gyro_radps[1], -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f);
  datalog_msg_.fmu_imu_gyro_z_radps = Scale(ref.sensor.fmu_imu.gyro_radps[2], -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f);

  /* FMU Mag */
  datalog_msg_.fmu_mag_healthy = ref.sensor.fmu_mag.healthy;
  datalog_msg_.fmu_mag_new_data = ref.sensor.fmu_mag.new_data;
  datalog_msg_.fmu_mag_die_temp_c = Scale(ref.sensor.fmu_mag.die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f);
  datalog_msg_.fmu_mag_x_ut = Scale(ref.sensor.fmu_mag.mag_ut[0], -250.0f, 250.0f, 131.07f, 32767.5f);
  datalog_msg_.fmu_mag_y_ut = Scale(ref.sensor.fmu_mag.mag_ut[1], -250.0f, 250.0f, 131.07f, 32767.5f);
  datalog_msg_.fmu_mag_z_ut = Scale(ref.sensor.fmu_mag.mag_ut[2], -250.0f, 250.0f, 131.07f, 32767.5f);

  /* FMU Static Pres */
  datalog_msg_.fmu_static_pres_healthy = ref.sensor.fmu_static_pres.healthy;
  datalog_msg_.fmu_static_pres_new_data = ref.sensor.fmu_static_pres.new_data;
  datalog_msg_.fmu_static_pres_die_temp_c = Scale(ref.sensor.fmu_static_pres.die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f);
  datalog_msg_.fmu_static_pres_pa = Scale(ref.sensor.fmu_static_pres.pres_pa, 60000.0f, 120000.0f, 1.09225f, -65535.0f);

  /* VectorNav */
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  datalog_msg_.vector_nav_installed = ref.sensor.vector_nav_imu.installed;
  datalog_msg_.vector_nav_healthy = ref.sensor.vector_nav_imu.healthy;
  datalog_msg_.vector_nav_die_temp_c = Scale(ref.sensor.vector_nav_imu.die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f);
  datalog_msg_.vector_nav_imu_new_data = ref.sensor.vector_nav_imu.new_data;
  datalog_msg_.vector_nav_imu_accel_x_mps2 = Scale(ref.sensor.vector_nav_imu.accel_mps2[0], -156.9064f, 156.9064f, 208.834693804714f, 32767.5f);
  datalog_msg_.vector_nav_imu_accel_y_mps2 = Scale(ref.sensor.vector_nav_imu.accel_mps2[1], -156.9064f, 156.9064f, 208.834693804714f, 32767.5f);
  datalog_msg_.vector_nav_imu_accel_z_mps2 = Scale(ref.sensor.vector_nav_imu.accel_mps2[2], -156.9064f, 156.9064f, 208.834693804714f, 32767.5f);
  datalog_msg_.vector_nav_imu_gyro_x_radps = Scale(ref.sensor.vector_nav_imu.gyro_radps[0], -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f);
  datalog_msg_.vector_nav_imu_gyro_y_radps = Scale(ref.sensor.vector_nav_imu.gyro_radps[1], -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f);
  datalog_msg_.vector_nav_imu_gyro_z_radps = Scale(ref.sensor.vector_nav_imu.gyro_radps[2], -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f);
  datalog_msg_.vector_nav_mag_x_ut = Scale(ref.sensor.vector_nav_mag.mag_ut[0], -250.0f, 250.0f, 131.07f, 32767.5f);
  datalog_msg_.vector_nav_mag_y_ut = Scale(ref.sensor.vector_nav_mag.mag_ut[1], -250.0f, 250.0f, 131.07f, 32767.5f);
  datalog_msg_.vector_nav_mag_z_ut = Scale(ref.sensor.vector_nav_mag.mag_ut[2], -250.0f, 250.0f, 131.07f, 32767.5f);
  datalog_msg_.vector_nav_static_pres_pa = Scale(ref.sensor.vector_nav_static_pres.pres_pa, 60000.0f, 120000.0f, 1.09225f, -65535.0f);
  datalog_msg_.vector_nav_gnss_healthy = ref.sensor.vector_nav_gnss.healthy;
  datalog_msg_.vector_nav_gnss_new_data = ref.sensor.vector_nav_gnss.new_data;
  datalog_msg_.vector_nav_gnss_fix = ref.sensor.vector_nav_gnss.fix;
  datalog_msg_.vector_nav_gnss_num_sats = ref.sensor.vector_nav_gnss.num_sats;
  datalog_msg_.vector_nav_gnss_gps_week = ref.sensor.vector_nav_gnss.gps_week;
  datalog_msg_.vector_nav_gnss_alt_wgs84 = Scale(ref.sensor.vector_nav_gnss.alt_wgs84_m, -500.0f, 5000.0f, 11.9154545454545f, 5957.72727272727f);
  datalog_msg_.vector_nav_gnss_horz_acc_m = Scale(ref.sensor.vector_nav_gnss.horz_acc_m, 0.0f, 10.0f, 1638.3f, 0.0f);
  datalog_msg_.vector_nav_gnss_vert_acc_m = Scale(ref.sensor.vector_nav_gnss.vert_acc_m, 0.0f, 10.0f, 1638.3f, 0.0f);
  datalog_msg_.vector_nav_gnss_vel_acc_mps = Scale(ref.sensor.vector_nav_gnss.vel_acc_mps, 0.0f, 10.0f, 1638.3f, 0.0f);
  datalog_msg_.vector_nav_gnss_north_vel_mps = Scale(ref.sensor.vector_nav_gnss.ned_vel_mps[0], -60.0f, 60.0f, 17.0583333333333f, 1023.5f);
  datalog_msg_.vector_nav_gnss_east_vel_mps = Scale(ref.sensor.vector_nav_gnss.ned_vel_mps[1], -60.0f, 60.0f, 17.0583333333333f, 1023.5f);
  datalog_msg_.vector_nav_gnss_down_vel_mps = Scale(ref.sensor.vector_nav_gnss.ned_vel_mps[2], -60.0f, 60.0f, 17.0583333333333f, 1023.5f);
  datalog_msg_.vector_nav_gnss_lat_rad = Scale(ref.sensor.vector_nav_gnss.lat_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 11930464.7083333, 2147483647.5);
  datalog_msg_.vector_nav_gnss_lon_rad = Scale(ref.sensor.vector_nav_gnss.lon_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 11930464.7083333, 2147483647.5);
  datalog_msg_.vector_nav_gnss_gps_tow_s = Scale(ref.sensor.vector_nav_gnss.gps_tow_s, 0.0, 604800.0, 1775.36677083333, 0.0);
  #endif

  /* EXT Mag */
  datalog_msg_.ext_mag_installed = ref.sensor.ext_mag.installed;
  datalog_msg_.ext_mag_healthy = ref.sensor.ext_mag.healthy;
  datalog_msg_.ext_mag_new_data = ref.sensor.ext_mag.new_data;
  datalog_msg_.ext_mag_die_temp_c = Scale(ref.sensor.ext_mag.die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f);
  datalog_msg_.ext_mag_x_ut = Scale(ref.sensor.ext_mag.mag_ut[0], -250.0f, 250.0f, 131.07f, 32767.5f);
  datalog_msg_.ext_mag_y_ut = Scale(ref.sensor.ext_mag.mag_ut[1], -250.0f, 250.0f, 131.07f, 32767.5f);
  datalog_msg_.ext_mag_z_ut = Scale(ref.sensor.ext_mag.mag_ut[2], -250.0f, 250.0f, 131.07f, 32767.5f);

  /* EXT GNSS 1 */
  datalog_msg_.ext_gnss1_installed = ref.sensor.ext_gnss1.installed;
  datalog_msg_.ext_gnss1_healthy = ref.sensor.ext_gnss1.healthy;
  datalog_msg_.ext_gnss1_new_data = ref.sensor.ext_gnss1.new_data;
  datalog_msg_.ext_gnss1_rel_pos_avail = ref.sensor.ext_gnss1.rel_pos_avail;
  datalog_msg_.ext_gnss1_rel_pos_moving_baseline = ref.sensor.ext_gnss1.rel_pos_moving_baseline;
  datalog_msg_.ext_gnss1_rel_pos_baseline_normalized = ref.sensor.ext_gnss1.rel_pos_baseline_normalized;
  datalog_msg_.ext_gnss1_fix = ref.sensor.ext_gnss1.fix;
  datalog_msg_.ext_gnss1_num_sats = ref.sensor.ext_gnss1.num_sats;
  datalog_msg_.ext_gnss1_gps_week = ref.sensor.ext_gnss1.gps_week;
  datalog_msg_.ext_gnss1_alt_wgs84_m = Scale(ref.sensor.ext_gnss1.alt_wgs84_m, -500.0f, 5000.0f, 190.65f, 95324.9999999999f);
  datalog_msg_.ext_gnss1_horz_acc_m = Scale(ref.sensor.ext_gnss1.horz_acc_m, 0.0f, 10.0f, 1638.3f, 0.0f);
  datalog_msg_.ext_gnss1_vert_acc_m = Scale(ref.sensor.ext_gnss1.vert_acc_m, 0.0f, 10.0f, 1638.3f, 0.0f);
  datalog_msg_.ext_gnss1_vel_acc_mps = Scale(ref.sensor.ext_gnss1.vel_acc_mps, 0.0f, 10.0f, 1638.3f, 0.0f);
  datalog_msg_.ext_gnss1_north_vel_mps = Scale(ref.sensor.ext_gnss1.ned_vel_mps[0], -60.0f, 60.0f, 136.525f, 8191.5f);
  datalog_msg_.ext_gnss1_east_vel_mps = Scale(ref.sensor.ext_gnss1.ned_vel_mps[1], -60.0f, 60.0f, 136.525f, 8191.5f);
  datalog_msg_.ext_gnss1_down_vel_mps = Scale(ref.sensor.ext_gnss1.ned_vel_mps[2], -60.0f, 60.0f, 136.525f, 8191.5f);
  datalog_msg_.ext_gnss1_lat_rad = Scale(ref.sensor.ext_gnss1.lat_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5);
  datalog_msg_.ext_gnss1_lon_rad = Scale(ref.sensor.ext_gnss1.lon_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5);
  datalog_msg_.ext_gnss1_gps_tow_s = Scale(ref.sensor.ext_gnss1.gps_tow_s, 0.0, 604800.0, 1775.36677083333, 0.0);
  datalog_msg_.ext_gnss1_rel_pos_acc_north_m = Scale(ref.sensor.ext_gnss1.rel_pos_acc_ned_m[0], 0.0f, 10.0f, 6553.5f, 0.0f);
  datalog_msg_.ext_gnss1_rel_pos_acc_east_m = Scale(ref.sensor.ext_gnss1.rel_pos_acc_ned_m[1], 0.0f, 10.0f, 6553.5f, 0.0f);
  datalog_msg_.ext_gnss1_rel_pos_acc_down_m = Scale(ref.sensor.ext_gnss1.rel_pos_acc_ned_m[2], 0.0f, 10.0f, 6553.5f, 0.0f);
  datalog_msg_.ext_gnss1_rel_pos_north_m = Scale(ref.sensor.ext_gnss1.rel_pos_ned_m[0], -50000.0, 50000.0, 1342.17727, 67108863.5);
  datalog_msg_.ext_gnss1_rel_pos_east_m = Scale(ref.sensor.ext_gnss1.rel_pos_ned_m[1], -50000.0, 50000.0, 1342.17727, 67108863.5);
  datalog_msg_.ext_gnss1_rel_pos_down_m = Scale(ref.sensor.ext_gnss1.rel_pos_ned_m[2], -5000.0, 500.0, 1525.20127272727, 7626006.36363636);

  /* EXT GNSS 2 */
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  datalog_msg_.ext_gnss2_installed = ref.sensor.ext_gnss2.installed;
  datalog_msg_.ext_gnss2_healthy = ref.sensor.ext_gnss2.healthy;
  datalog_msg_.ext_gnss2_new_data = ref.sensor.ext_gnss2.new_data;
  datalog_msg_.ext_gnss2_rel_pos_avail = ref.sensor.ext_gnss2.rel_pos_avail;
  datalog_msg_.ext_gnss2_rel_pos_moving_baseline = ref.sensor.ext_gnss2.rel_pos_moving_baseline;
  datalog_msg_.ext_gnss2_rel_pos_baseline_normalized = ref.sensor.ext_gnss2.rel_pos_baseline_normalized;
  datalog_msg_.ext_gnss2_fix = ref.sensor.ext_gnss2.fix;
  datalog_msg_.ext_gnss2_num_sats = ref.sensor.ext_gnss2.num_sats;
  datalog_msg_.ext_gnss2_gps_week = ref.sensor.ext_gnss2.gps_week;
  datalog_msg_.ext_gnss2_alt_wgs84_m = Scale(ref.sensor.ext_gnss2.alt_wgs84_m, -500.0f, 5000.0f, 190.65f, 95324.9999999999f);
  datalog_msg_.ext_gnss2_horz_acc_m = Scale(ref.sensor.ext_gnss2.horz_acc_m, 0.0f, 10.0f, 1638.3f, 0.0f);
  datalog_msg_.ext_gnss2_vert_acc_m = Scale(ref.sensor.ext_gnss2.vert_acc_m, 0.0f, 10.0f, 1638.3f, 0.0f);
  datalog_msg_.ext_gnss2_vel_acc_mps = Scale(ref.sensor.ext_gnss2.vel_acc_mps, 0.0f, 10.0f, 1638.3f, 0.0f);
  datalog_msg_.ext_gnss2_north_vel_mps = Scale(ref.sensor.ext_gnss2.ned_vel_mps[0], -60.0f, 60.0f, 136.525f, 8191.5f);
  datalog_msg_.ext_gnss2_east_vel_mps = Scale(ref.sensor.ext_gnss2.ned_vel_mps[1], -60.0f, 60.0f, 136.525f, 8191.5f);
  datalog_msg_.ext_gnss2_down_vel_mps = Scale(ref.sensor.ext_gnss2.ned_vel_mps[2], -60.0f, 60.0f, 136.525f, 8191.5f);
  datalog_msg_.ext_gnss2_lat_rad = Scale(ref.sensor.ext_gnss2.lat_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5);
  datalog_msg_.ext_gnss2_lon_rad = Scale(ref.sensor.ext_gnss2.lon_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5);
  datalog_msg_.ext_gnss2_gps_tow_s = Scale(ref.sensor.ext_gnss2.gps_tow_s, 0.0, 604800.0, 1775.36677083333, 0.0);
  datalog_msg_.ext_gnss2_rel_pos_acc_north_m = Scale(ref.sensor.ext_gnss2.rel_pos_acc_ned_m[0], 0.0f, 10.0f, 6553.5f, 0.0f);
  datalog_msg_.ext_gnss2_rel_pos_acc_east_m = Scale(ref.sensor.ext_gnss2.rel_pos_acc_ned_m[1], 0.0f, 10.0f, 6553.5f, 0.0f);
  datalog_msg_.ext_gnss2_rel_pos_acc_down_m = Scale(ref.sensor.ext_gnss2.rel_pos_acc_ned_m[2], 0.0f, 10.0f, 6553.5f, 0.0f);
  datalog_msg_.ext_gnss2_rel_pos_north_m = Scale(ref.sensor.ext_gnss2.rel_pos_ned_m[0], -50000.0, 50000.0, 1342.17727, 67108863.5);
  datalog_msg_.ext_gnss2_rel_pos_east_m = Scale(ref.sensor.ext_gnss2.rel_pos_ned_m[1], -50000.0, 50000.0, 1342.17727, 67108863.5);
  datalog_msg_.ext_gnss2_rel_pos_down_m = Scale(ref.sensor.ext_gnss2.rel_pos_ned_m[2], -5000.0, 500.0, 1525.20127272727, 7626006.36363636);
  #endif

  /* EXT Pres 1 */
  datalog_msg_.ext_pres1_installed = ref.sensor.ext_pres1.installed;
  datalog_msg_.ext_pres1_is_static_pres = ref.sensor.ext_pres1.is_static_pres;
  datalog_msg_.ext_pres1_healthy = ref.sensor.ext_pres1.healthy;
  datalog_msg_.ext_pres1_new_data = ref.sensor.ext_pres1.new_data;
  datalog_msg_.ext_pres1_die_temp_c = Scale(ref.sensor.ext_pres1.die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f);
  if (ref.sensor.ext_pres1.is_static_pres) {
    datalog_msg_.ext_pres1_pres_pa = Scale(ref.sensor.ext_pres1.pres_pa, 60000.0f, 120000.0f, 1.09225f, -65535.0f);
  } else {
    datalog_msg_.ext_pres1_pres_pa = Scale(ref.sensor.ext_pres1.pres_pa, -2000.0f, 2000.0f, 16.38375f, 32767.5f);
  }

  /* EXT Pres 2 */
  datalog_msg_.ext_pres2_installed = ref.sensor.ext_pres2.installed;
  datalog_msg_.ext_pres2_is_static_pres = ref.sensor.ext_pres2.is_static_pres;
  datalog_msg_.ext_pres2_healthy = ref.sensor.ext_pres2.healthy;
  datalog_msg_.ext_pres2_new_data = ref.sensor.ext_pres2.new_data;
  datalog_msg_.ext_pres2_die_temp_c = Scale(ref.sensor.ext_pres2.die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f);
  if (ref.sensor.ext_pres2.is_static_pres) {
    datalog_msg_.ext_pres2_pres_pa = Scale(ref.sensor.ext_pres2.pres_pa, 60000.0f, 120000.0f, 1.09225f, -65535.0f);
  } else {
    datalog_msg_.ext_pres2_pres_pa = Scale(ref.sensor.ext_pres2.pres_pa, -2000.0f, 2000.0f, 16.38375f, 32767.5f);
  }

  /* EXT Pres 3 */
  datalog_msg_.ext_pres3_installed = ref.sensor.ext_pres3.installed;
  datalog_msg_.ext_pres3_is_static_pres = ref.sensor.ext_pres3.is_static_pres;
  datalog_msg_.ext_pres3_healthy = ref.sensor.ext_pres3.healthy;
  datalog_msg_.ext_pres3_new_data = ref.sensor.ext_pres3.new_data;
  datalog_msg_.ext_pres3_die_temp_c = Scale(ref.sensor.ext_pres3.die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f);
  if (ref.sensor.ext_pres3.is_static_pres) {
    datalog_msg_.ext_pres3_pres_pa = Scale(ref.sensor.ext_pres3.pres_pa, 60000.0f, 120000.0f, 1.09225f, -65535.0f);
  } else {
    datalog_msg_.ext_pres3_pres_pa = Scale(ref.sensor.ext_pres3.pres_pa, -2000.0f, 2000.0f, 16.38375f, 32767.5f);
  }

  /* EXT Pres 4 */
  datalog_msg_.ext_pres4_installed = ref.sensor.ext_pres4.installed;
  datalog_msg_.ext_pres4_is_static_pres = ref.sensor.ext_pres4.is_static_pres;
  datalog_msg_.ext_pres4_healthy = ref.sensor.ext_pres4.healthy;
  datalog_msg_.ext_pres4_new_data = ref.sensor.ext_pres4.new_data;
  datalog_msg_.ext_pres4_die_temp_c = Scale(ref.sensor.ext_pres4.die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f);
  if (ref.sensor.ext_pres4.is_static_pres) {
    datalog_msg_.ext_pres4_pres_pa = Scale(ref.sensor.ext_pres4.pres_pa, 60000.0f, 120000.0f, 1.09225f, -65535.0f);
  } else {
    datalog_msg_.ext_pres4_pres_pa = Scale(ref.sensor.ext_pres4.pres_pa, -2000.0f, 2000.0f, 16.38375f, 32767.5f);
  }

  /* RAD ALT */
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  datalog_msg_.rad_alt_installed = ref.sensor.rad_alt.installed;
  datalog_msg_.rad_alt_healthy = ref.sensor.rad_alt.healthy;
  datalog_msg_.rad_alt_new_data = ref.sensor.rad_alt.new_data;
  datalog_msg_.rad_alt_snr = ref.sensor.rad_alt.snr;
  datalog_msg_.rad_alt_alt_m = Scale(ref.sensor.rad_alt.alt_m, 0.0f, 50.0f, 40.94f, 0.0f);
  #endif
  

  /* OPTICAL FLOW */
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  datalog_msg_.opflow_installed = ref.sensor.opflow.installed;
  datalog_msg_.opflow_healthy = ref.sensor.opflow.healthy;
  datalog_msg_.opflow_new_data = ref.sensor.opflow.new_data;
  datalog_msg_.opflow_sur_qual = ref.sensor.opflow.sur_qual;
  datalog_msg_.opflow_range_qual = ref.sensor.opflow.range_qual;
  datalog_msg_.opflow_mot_x = Scale(float(ref.sensor.opflow.mot_x), -500.0f, 500.0f, 1.0f, 600.0f);
  datalog_msg_.opflow_mot_y = Scale(float(ref.sensor.opflow.mot_y), -500.0f, 500.0f, 1.0f, 600.0f);
  datalog_msg_.opflow_range_mm = Scale(float(ref.sensor.opflow.range_mm), -1.0f, 2000.0f, 1.0f, 3000.0f);
  #endif

  /* AIN */
  datalog_msg_.ain0_v = Scale(ref.sensor.analog.voltage_v[0], 0.0f, 3.3f, 1240.90909090909f, 0.0f);
  datalog_msg_.ain1_v = Scale(ref.sensor.analog.voltage_v[1], 0.0f, 3.3f, 1240.90909090909f, 0.0f);
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  datalog_msg_.ain2_v = Scale(ref.sensor.analog.voltage_v[2], 0.0f, 3.3f, 1240.90909090909f, 0.0f);
  datalog_msg_.ain3_v = Scale(ref.sensor.analog.voltage_v[3], 0.0f, 3.3f, 1240.90909090909f, 0.0f);
  datalog_msg_.ain4_v = Scale(ref.sensor.analog.voltage_v[4], 0.0f, 3.3f, 1240.90909090909f, 0.0f);
  datalog_msg_.ain5_v = Scale(ref.sensor.analog.voltage_v[5], 0.0f, 3.3f, 1240.90909090909f, 0.0f);
  #endif
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__)
  datalog_msg_.ain6_v = Scale(ref.sensor.analog.voltage_v[6], 0.0f, 3.3f, 1240.90909090909f, 0.0f);
  datalog_msg_.ain7_v = Scale(ref.sensor.analog.voltage_v[7], 0.0f, 3.3f, 1240.90909090909f, 0.0f);
  #endif

  /* PWR MODULE */
  #if defined(__FMU_R_V2__) || defined(__FMU_R_MINI_V1__)
  datalog_msg_.power_module_voltage_v = Scale(ref.sensor.power_module.voltage_v, 0.0f, 90.0f, 45.5f, 0.0f);
  datalog_msg_.power_module_current_ma = Scale(ref.sensor.power_module.current_ma, 0.0f, 180000.0f, 1.45635f, 0.0f);
  #endif

  /* BFS INS */
  datalog_msg_.bfs_ins_initialized = ref.state_est.bfs_ins.initialized;
  datalog_msg_.bfs_ins_pitch_rad = Scale(ref.state_est.bfs_ins.pitch_rad, -bfs::BFS_PI<float> / 2.0f, bfs::BFS_PI<float> / 2.0f, 20860.4383910547f, 32767.5f);
  datalog_msg_.bfs_ins_roll_rad = Scale(ref.state_est.bfs_ins.roll_rad, -bfs::BFS_PI<float> / 2.0f, bfs::BFS_PI<float> / 2.0f, 20860.4383910547f, 32767.5f);
  datalog_msg_.bfs_ins_heading_rad = Scale(ref.state_est.bfs_ins.pitch_rad, -bfs::BFS_PI<float>, bfs::BFS_PI<float>, 10430.2191955274f, 32767.5f);
  datalog_msg_.bfs_ins_alt_wgs84_m = Scale(ref.state_est.bfs_ins.alt_wgs84_m, -500.0f, 5000.0f, 190.65f, 95324.9999999999f);
  datalog_msg_.bfs_ins_accel_x_mps2 = Scale(ref.state_est.bfs_ins.accel_mps2[0], -156.9064f, 156.9064f, 208.834693804714f, 32767.5f);
  datalog_msg_.bfs_ins_accel_y_mps2 = Scale(ref.state_est.bfs_ins.accel_mps2[1], -156.9064f, 156.9064f, 208.834693804714f, 32767.5f);
  datalog_msg_.bfs_ins_accel_z_mps2 = Scale(ref.state_est.bfs_ins.accel_mps2[2], -156.9064f, 156.9064f, 208.834693804714f, 32767.5f);
  datalog_msg_.bfs_ins_gyro_x_radps = Scale(ref.state_est.bfs_ins.gyro_radps[0], -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f);
  datalog_msg_.bfs_ins_gyro_y_radps = Scale(ref.state_est.bfs_ins.gyro_radps[1], -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f);
  datalog_msg_.bfs_ins_gyro_z_radps = Scale(ref.state_est.bfs_ins.gyro_radps[2], -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f);
  datalog_msg_.bfs_ins_mag_x_ut = Scale(ref.state_est.bfs_ins.mag_ut[0], -250.0f, 250.0f, 131.07f, 32767.5f);
  datalog_msg_.bfs_ins_mag_y_ut = Scale(ref.state_est.bfs_ins.mag_ut[1], -250.0f, 250.0f, 131.07f, 32767.5f);
  datalog_msg_.bfs_ins_mag_z_ut = Scale(ref.state_est.bfs_ins.mag_ut[2], -250.0f, 250.0f, 131.07f, 32767.5f);
  datalog_msg_.bfs_ins_north_vel_mps = Scale(ref.state_est.bfs_ins.ned_vel_mps[0], -60.0f, 60.0f, 136.525f, 8191.5f);
  datalog_msg_.bfs_ins_east_vel_mps = Scale(ref.state_est.bfs_ins.ned_vel_mps[1], -60.0f, 60.0f, 136.525f, 8191.5f);
  datalog_msg_.bfs_ins_down_vel_mps = Scale(ref.state_est.bfs_ins.ned_vel_mps[2], -60.0f, 60.0f, 136.525f, 8191.5f);
  datalog_msg_.bfs_ins_lat_rad = Scale(ref.state_est.bfs_ins.lat_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5);
  datalog_msg_.bfs_ins_lon_rad = Scale(ref.state_est.bfs_ins.lon_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5);

  /* VectorNav INS */
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  datalog_msg_.vector_nav_ins_initialized = ref.state_est.vector_nav_ins.initialized;
  datalog_msg_.vector_nav_ins_pitch_rad = Scale(ref.state_est.vector_nav_ins.pitch_rad, -bfs::BFS_PI<float> / 2.0f, bfs::BFS_PI<float> / 2.0f, 20860.4383910547f, 32767.5f);
  datalog_msg_.vector_nav_ins_roll_rad = Scale(ref.state_est.vector_nav_ins.roll_rad, -bfs::BFS_PI<float> / 2.0f, bfs::BFS_PI<float> / 2.0f, 20860.4383910547f, 32767.5f);
  datalog_msg_.vector_nav_ins_heading_rad = Scale(ref.state_est.vector_nav_ins.pitch_rad, -bfs::BFS_PI<float>, bfs::BFS_PI<float>, 10430.2191955274f, 32767.5f);
  datalog_msg_.vector_nav_ins_alt_wgs84_m = Scale(ref.state_est.vector_nav_ins.alt_wgs84_m, -500.0f, 5000.0f, 190.65f, 95324.9999999999f);
  datalog_msg_.vector_nav_ins_accel_x_mps2 = Scale(ref.state_est.vector_nav_ins.accel_mps2[0], -156.9064f, 156.9064f, 208.834693804714f, 32767.5f);
  datalog_msg_.vector_nav_ins_accel_y_mps2 = Scale(ref.state_est.vector_nav_ins.accel_mps2[1], -156.9064f, 156.9064f, 208.834693804714f, 32767.5f);
  datalog_msg_.vector_nav_ins_accel_z_mps2 = Scale(ref.state_est.vector_nav_ins.accel_mps2[2], -156.9064f, 156.9064f, 208.834693804714f, 32767.5f);
  datalog_msg_.vector_nav_ins_gyro_x_radps = Scale(ref.state_est.vector_nav_ins.gyro_radps[0], -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f);
  datalog_msg_.vector_nav_ins_gyro_y_radps = Scale(ref.state_est.vector_nav_ins.gyro_radps[1], -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f);
  datalog_msg_.vector_nav_ins_gyro_z_radps = Scale(ref.state_est.vector_nav_ins.gyro_radps[2], -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f);
  datalog_msg_.vector_nav_ins_mag_x_ut = Scale(ref.state_est.vector_nav_ins.mag_ut[0], -250.0f, 250.0f, 131.07f, 32767.5f);
  datalog_msg_.vector_nav_ins_mag_y_ut = Scale(ref.state_est.vector_nav_ins.mag_ut[1], -250.0f, 250.0f, 131.07f, 32767.5f);
  datalog_msg_.vector_nav_ins_mag_z_ut = Scale(ref.state_est.vector_nav_ins.mag_ut[2], -250.0f, 250.0f, 131.07f, 32767.5f);
  datalog_msg_.vector_nav_ins_north_vel_mps = Scale(ref.state_est.vector_nav_ins.ned_vel_mps[0], -60.0f, 60.0f, 136.525f, 8191.5f);
  datalog_msg_.vector_nav_ins_east_vel_mps = Scale(ref.state_est.vector_nav_ins.ned_vel_mps[1], -60.0f, 60.0f, 136.525f, 8191.5f);
  datalog_msg_.vector_nav_ins_down_vel_mps = Scale(ref.state_est.vector_nav_ins.ned_vel_mps[2], -60.0f, 60.0f, 136.525f, 8191.5f);
  datalog_msg_.vector_nav_ins_lat_rad = Scale(ref.state_est.vector_nav_ins.lat_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5);
  datalog_msg_.vector_nav_ins_lon_rad = Scale(ref.state_est.vector_nav_ins.lon_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5);
  #endif

  /* AUX INS data */
  datalog_msg_.aux_ins_home_alt_wgs84_m = Scale(ref.state_est.aux_ins.home_alt_wgs84_m, -500.0f, 5000.0f, 190.65f, 95324.9999999999f);
  datalog_msg_.aux_ins_gnd_spd_mps = Scale(ref.state_est.aux_ins.gnd_spd_mps, 0.0f, 60.0f, 136.516666666667f, 0.0f);
  datalog_msg_.aux_ins_gnd_track_rad = Scale(ref.state_est.aux_ins.gnd_track_rad, -bfs::BFS_PI<float>, bfs::BFS_PI<float>, 10430.2191955274f, 32767.5f);
  datalog_msg_.aux_ins_flight_path_rad = Scale(ref.state_est.aux_ins.flight_path_rad, -bfs::BFS_PI<float> / 2.0f, bfs::BFS_PI<float> / 2.0f, 10430.0600405843f, 16383.5f);
  datalog_msg_.aux_ins_home_lat_rad = Scale(ref.state_est.aux_ins.home_lat_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5);
  datalog_msg_.aux_ins_home_lon_rad = Scale(ref.state_est.aux_ins.home_lon_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5);
  datalog_msg_.aux_ins_ned_pos_north_m = Scale(ref.state_est.aux_ins.ned_pos_m[0], -50000.0, 50000.0, 1342.17727, 67108863.5);
  datalog_msg_.aux_ins_ned_pos_east_m = Scale(ref.state_est.aux_ins.ned_pos_m[1], -50000.0, 50000.0, 1342.17727, 67108863.5);
  datalog_msg_.aux_ins_ned_pos_down_m = Scale(ref.state_est.aux_ins.ned_pos_m[2], -5000.0, 500.0, 1525.20127272727, 7626006.36363636);

  /* ADC */
  datalog_msg_.adc_static_pres_pa = Scale(ref.state_est.adc.static_pres_pa, 60000.0f, 120000.0f, 1.09225f, -65535.0f);
  datalog_msg_.adc_diff_pres_pa = Scale(ref.state_est.adc.diff_pres_pa, 0.0f, 2000.0f, 32.7675f, 0.0f);
  datalog_msg_.adc_pres_alt_m = Scale(ref.state_est.adc.pres_alt_m, -500.0f, 5000.0f, 11.9154545454545f, 5957.72727272727f);
  datalog_msg_.adc_rel_alt_m = Scale(ref.state_est.adc.rel_alt_m, -500.0f, 5000.0f, 11.9154545454545f, 5957.72727272727f);
  datalog_msg_.adc_ias_mps = Scale(ref.state_est.adc.ias_mps, 0.0f, 60.0f, 17.05f, 0.0f);

  /* TELEM */
  datalog_msg_.telem_waypoint_frame = ref.telem.flight_plan[ref.telem.current_waypoint].frame;
  datalog_msg_.telem_waypoint_cmd = ref.telem.flight_plan[ref.telem.current_waypoint].cmd;
  memcpy(&temp_, &ref.telem.flight_plan[ref.telem.current_waypoint].param1, 4);
  datalog_msg_.telem_waypoint_param1 = temp_;
  memcpy(&temp_, &ref.telem.flight_plan[ref.telem.current_waypoint].param2, 4);
  datalog_msg_.telem_waypoint_param2 = temp_;
  memcpy(&temp_, &ref.telem.flight_plan[ref.telem.current_waypoint].param3, 4);
  datalog_msg_.telem_waypoint_param3 = temp_;
  memcpy(&temp_, &ref.telem.flight_plan[ref.telem.current_waypoint].param4, 4);
  datalog_msg_.telem_waypoint_param4 = temp_;
  memcpy(&temp_, &ref.telem.flight_plan[ref.telem.current_waypoint].x, 4);
  datalog_msg_.telem_waypoint_x = temp_;
  memcpy(&temp_, &ref.telem.flight_plan[ref.telem.current_waypoint].y, 4);
  datalog_msg_.telem_waypoint_y = temp_;
  memcpy(&temp_, &ref.telem.flight_plan[ref.telem.current_waypoint].z, 4);
  datalog_msg_.telem_waypoint_z = temp_;
  memcpy(&temp_, &ref.telem.param[0], 4);
  datalog_msg_.telem_param0 = temp_;

  memcpy(&temp_, &ref.telem.param[1], 4);
  datalog_msg_.telem_param1 = temp_;

  memcpy(&temp_, &ref.telem.param[2], 4);
  datalog_msg_.telem_param2 = temp_;

  memcpy(&temp_, &ref.telem.param[3], 4);
  datalog_msg_.telem_param3 = temp_;

  memcpy(&temp_, &ref.telem.param[4], 4);
  datalog_msg_.telem_param4 = temp_;

  memcpy(&temp_, &ref.telem.param[5], 4);
  datalog_msg_.telem_param5 = temp_;

  memcpy(&temp_, &ref.telem.param[6], 4);
  datalog_msg_.telem_param6 = temp_;

  memcpy(&temp_, &ref.telem.param[7], 4);
  datalog_msg_.telem_param7 = temp_;

  memcpy(&temp_, &ref.telem.param[8], 4);
  datalog_msg_.telem_param8 = temp_;

  memcpy(&temp_, &ref.telem.param[9], 4);
  datalog_msg_.telem_param9 = temp_;

  memcpy(&temp_, &ref.telem.param[10], 4);
  datalog_msg_.telem_param10 = temp_;

  memcpy(&temp_, &ref.telem.param[11], 4);
  datalog_msg_.telem_param11 = temp_;

  memcpy(&temp_, &ref.telem.param[12], 4);
  datalog_msg_.telem_param12 = temp_;

  memcpy(&temp_, &ref.telem.param[13], 4);
  datalog_msg_.telem_param13 = temp_;

  memcpy(&temp_, &ref.telem.param[14], 4);
  datalog_msg_.telem_param14 = temp_;

  memcpy(&temp_, &ref.telem.param[15], 4);
  datalog_msg_.telem_param15 = temp_;

  memcpy(&temp_, &ref.telem.param[16], 4);
  datalog_msg_.telem_param16 = temp_;

  memcpy(&temp_, &ref.telem.param[17], 4);
  datalog_msg_.telem_param17 = temp_;

  memcpy(&temp_, &ref.telem.param[18], 4);
  datalog_msg_.telem_param18 = temp_;

  memcpy(&temp_, &ref.telem.param[19], 4);
  datalog_msg_.telem_param19 = temp_;

  memcpy(&temp_, &ref.telem.param[20], 4);
  datalog_msg_.telem_param20 = temp_;

  memcpy(&temp_, &ref.telem.param[21], 4);
  datalog_msg_.telem_param21 = temp_;

  memcpy(&temp_, &ref.telem.param[22], 4);
  datalog_msg_.telem_param22 = temp_;

  memcpy(&temp_, &ref.telem.param[23], 4);
  datalog_msg_.telem_param23 = temp_;

  /* VMS */
  datalog_msg_.vms_advance_waypoint = ref.vms.advance_waypoint;
  datalog_msg_.vms_motors_enabled = ref.vms.motors_enabled;
  datalog_msg_.vms_mode = ref.vms.mode;
  datalog_msg_.vms_sbus_cmd0 = ref.vms.sbus[0];
  datalog_msg_.vms_sbus_cmd1 = ref.vms.sbus[1];
  datalog_msg_.vms_sbus_cmd2 = ref.vms.sbus[2];
  datalog_msg_.vms_sbus_cmd3 = ref.vms.sbus[3];
  datalog_msg_.vms_sbus_cmd4 = ref.vms.sbus[4];
  datalog_msg_.vms_sbus_cmd5 = ref.vms.sbus[5];
  datalog_msg_.vms_sbus_cmd6 = ref.vms.sbus[6];
  datalog_msg_.vms_sbus_cmd7 = ref.vms.sbus[7];
  datalog_msg_.vms_sbus_cmd8 = ref.vms.sbus[8];
  datalog_msg_.vms_sbus_cmd9 = ref.vms.sbus[9];
  datalog_msg_.vms_sbus_cmd10 = ref.vms.sbus[10];
  datalog_msg_.vms_sbus_cmd11 = ref.vms.sbus[11];
  datalog_msg_.vms_sbus_cmd12 = ref.vms.sbus[12];
  datalog_msg_.vms_sbus_cmd13 = ref.vms.sbus[13];
  datalog_msg_.vms_sbus_cmd14 = ref.vms.sbus[14];
  datalog_msg_.vms_sbus_cmd15 = ref.vms.sbus[15];
  datalog_msg_.vms_pwm_cmd0 = ref.vms.pwm[0] - 1000;
  datalog_msg_.vms_pwm_cmd1 = ref.vms.pwm[1] - 1000;
  datalog_msg_.vms_pwm_cmd2 = ref.vms.pwm[2] - 1000;
  datalog_msg_.vms_pwm_cmd3 = ref.vms.pwm[3] - 1000;
  datalog_msg_.vms_pwm_cmd4 = ref.vms.pwm[4] - 1000;
  datalog_msg_.vms_pwm_cmd5 = ref.vms.pwm[5] - 1000;
  datalog_msg_.vms_pwm_cmd6 = ref.vms.pwm[6] - 1000;
  datalog_msg_.vms_pwm_cmd7 = ref.vms.pwm[7] - 1000;
  datalog_msg_.vms_flight_time_remaining_s = Scale(ref.vms.flight_time_remaining_s, 0.0f, 32400.0f, 2.02268518518519f, 0.0f);
  datalog_msg_.vms_power_remaining_prcnt = Scale(ref.vms.power_remaining_prcnt, 0.0f, 100.0f, 10.23f, 0.0f);
  datalog_msg_.vms_throttle_prcnt = Scale(ref.vms.throttle_cmd_prcnt, 0.0f, 100.0f, 10.23f, 0.0f);
  memcpy(&temp_, &ref.vms.aux[0], 4);
  datalog_msg_.vms_aux0 = temp_;
  memcpy(&temp_, &ref.vms.aux[1], 4);
  datalog_msg_.vms_aux1 = temp_;
  memcpy(&temp_, &ref.vms.aux[2], 4);
  datalog_msg_.vms_aux2 = temp_;
  memcpy(&temp_, &ref.vms.aux[3], 4);
  datalog_msg_.vms_aux3 = temp_;
  memcpy(&temp_, &ref.vms.aux[4], 4);
  datalog_msg_.vms_aux4 = temp_;
  memcpy(&temp_, &ref.vms.aux[5], 4);
  datalog_msg_.vms_aux5 = temp_;
  memcpy(&temp_, &ref.vms.aux[6], 4);
  datalog_msg_.vms_aux6 = temp_;
  memcpy(&temp_, &ref.vms.aux[7], 4);
  datalog_msg_.vms_aux7 = temp_;
  memcpy(&temp_, &ref.vms.aux[8], 4);
  datalog_msg_.vms_aux8 = temp_;
  memcpy(&temp_, &ref.vms.aux[9], 4);
  datalog_msg_.vms_aux9 = temp_;
  memcpy(&temp_, &ref.vms.aux[10], 4);
  datalog_msg_.vms_aux10 = temp_;
  memcpy(&temp_, &ref.vms.aux[11], 4);
  datalog_msg_.vms_aux11 = temp_;
  memcpy(&temp_, &ref.vms.aux[12], 4);
  datalog_msg_.vms_aux12 = temp_;
  memcpy(&temp_, &ref.vms.aux[13], 4);
  datalog_msg_.vms_aux13 = temp_;
  memcpy(&temp_, &ref.vms.aux[14], 4);
  datalog_msg_.vms_aux14 = temp_;
  memcpy(&temp_, &ref.vms.aux[15], 4);
  datalog_msg_.vms_aux15 = temp_;
  memcpy(&temp_, &ref.vms.aux[16], 4);
  datalog_msg_.vms_aux16 = temp_;
  memcpy(&temp_, &ref.vms.aux[17], 4);
  datalog_msg_.vms_aux17 = temp_;
  memcpy(&temp_, &ref.vms.aux[18], 4);
  datalog_msg_.vms_aux18 = temp_;
  memcpy(&temp_, &ref.vms.aux[19], 4);
  datalog_msg_.vms_aux19 = temp_;
  memcpy(&temp_, &ref.vms.aux[20], 4);
  datalog_msg_.vms_aux20 = temp_;
  memcpy(&temp_, &ref.vms.aux[21], 4);
  datalog_msg_.vms_aux21 = temp_;
  memcpy(&temp_, &ref.vms.aux[22], 4);
  datalog_msg_.vms_aux22 = temp_;
  memcpy(&temp_, &ref.vms.aux[23], 4);
  datalog_msg_.vms_aux23 = temp_;

  /* Framing */
  std::size_t bytes_written = encoder.Write((uint8_t *)&datalog_msg_,
                                            sizeof(datalog_msg_));
  if (bytes_written != sizeof(datalog_msg_)) {
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

void WaypointWrite(const AircraftData &ref){
  /* 
  Add waypoint to SD card for persistent mission
  Write all parameters in chunks of 4 bytes
  */
  sd_.remove("waypoints.txt");
  File32 wpfile_ = sd_.open("waypoints.txt", O_WRITE | O_CREAT);
  wp_param_type wp_param_;
  wp_param_.i = ref.telem.num_waypoints;
  write_union(wp_param_.bytes, wpfile_);
  for (uint16_t i = 0; i < ref.telem.num_waypoints; i++){
    // WP frame
    wp_param_.i = ref.telem.flight_plan[i].frame;
    write_union(wp_param_.bytes, wpfile_);
    // WP cmd
    wp_param_.i = ref.telem.flight_plan[i].cmd;
    write_union(wp_param_.bytes, wpfile_);
    // WP x
    wp_param_.i = ref.telem.flight_plan[i].x;
    write_union(wp_param_.bytes, wpfile_);
    // WP y
    wp_param_.i = ref.telem.flight_plan[i].y;
    write_union(wp_param_.bytes, wpfile_);
    // WP z
    wp_param_.f = ref.telem.flight_plan[i].z;
    write_union(wp_param_.bytes, wpfile_);
  }
  wpfile_.close();
}

void WaypointRead(TelemData * const ptr){
  MsgInfo("Updating waypoints with saved flight plan...");
  File32 wpfile_ = sd_.open("waypoints.txt", O_READ);
  if (!wpfile_){
    MsgInfo("No waypoint file preloaded. Upload waypoint to generate waypoint file.\n");
    return;
  }
  wp_param_type cur_chunk_;
  read_union(&cur_chunk_, wpfile_);
  uint16_t num_waypoints = cur_chunk_.i;
  ptr->waypoints_updated = true;
  ptr->current_waypoint = 0;
  ptr->num_waypoints = num_waypoints;
  for (uint16_t i = 0; i < num_waypoints; i++){
    read_union(&cur_chunk_, wpfile_);
    ptr->flight_plan[i].frame = cur_chunk_.i;
    read_union(&cur_chunk_, wpfile_);
    ptr->flight_plan[i].cmd = cur_chunk_.i;
    read_union(&cur_chunk_, wpfile_);
    ptr->flight_plan[i].x = cur_chunk_.i;
    read_union(&cur_chunk_, wpfile_);
    ptr->flight_plan[i].y = cur_chunk_.i;
    read_union(&cur_chunk_, wpfile_);
    ptr->flight_plan[i].z = cur_chunk_.f;
  }
  wpfile_.close();
  Mission_Num_Update(ptr->num_waypoints);
  MsgInfo("done.\n");
}

void write_union(uint8_t byte[4], File32 &file_){
  /*
  Function to write 4 bytes to a file. For convenience
  */
  for (uint8_t i = 0; i<4; i++){
    file_.write(byte[i]);
  }
}

void read_union(wp_param_type * const ptr, File32 &file_){
  /*
  Function to read 4 bytes from a file and append that to the union byte_array.
  */
  for (int8_t i = 0; i<4; i++){
    ptr->bytes[i] = file_.read();
  }
}
