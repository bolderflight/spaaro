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

#include <stdio.h>
#include <iostream>
#include <vector>
#include "framing.h"
#include "mat_v4/mat_v4.h"
#include "units.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "../../common/datalog_fmu.h"

std::vector<double> sys_frame_time_s;
#if defined(__FMU_R_V1__)
std::vector<float> sys_input_volt;
std::vector<float> sys_reg_volt;
std::vector<float> sys_pwm_volt;
std::vector<float> sys_sbus_volt;
#endif
std::vector<double> sys_time_s;
std::vector<uint8_t> incept_new_data;
std::vector<uint8_t> incept_lost_frame;
std::vector<uint8_t> incept_failsafe;
std::vector<int16_t> incept_ch0;
std::vector<int16_t> incept_ch1;
std::vector<int16_t> incept_ch2;
std::vector<int16_t> incept_ch3;
std::vector<int16_t> incept_ch4;
std::vector<int16_t> incept_ch5;
std::vector<int16_t> incept_ch6;
std::vector<int16_t> incept_ch7;
std::vector<int16_t> incept_ch8;
std::vector<int16_t> incept_ch9;
std::vector<int16_t> incept_ch10;
std::vector<int16_t> incept_ch11;
std::vector<int16_t> incept_ch12;
std::vector<int16_t> incept_ch13;
std::vector<int16_t> incept_ch14;
std::vector<int16_t> incept_ch15;
std::vector<uint8_t> fmu_imu_healthy;
std::vector<uint8_t> fmu_imu_new_data;
std::vector<float> fmu_imu_die_temp_c;
std::vector<float> fmu_imu_accel_x_mps2;
std::vector<float> fmu_imu_accel_y_mps2;
std::vector<float> fmu_imu_accel_z_mps2;
std::vector<float> fmu_imu_gyro_x_radps;
std::vector<float> fmu_imu_gyro_y_radps;
std::vector<float> fmu_imu_gyro_z_radps;
std::vector<uint8_t> fmu_mag_healthy;
std::vector<uint8_t> fmu_mag_new_data;
std::vector<float> fmu_mag_die_temp_c;
std::vector<float> fmu_mag_x_ut;
std::vector<float> fmu_mag_y_ut;
std::vector<float> fmu_mag_z_ut;
std::vector<uint8_t> fmu_static_pres_healthy;
std::vector<uint8_t> fmu_static_pres_new_data;
std::vector<float> fmu_static_pres_die_temp_c;
std::vector<float> fmu_static_pres_pa;
#if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
    defined(__FMU_R_V2_BETA__)
std::vector<uint8_t> vector_nav_installed;
std::vector<uint8_t> vector_nav_healthy;
std::vector<float> vector_nav_die_temp_c;
std::vector<uint8_t> vector_nav_imu_new_data;
std::vector<float> vector_nav_imu_accel_x_mps2;
std::vector<float> vector_nav_imu_accel_y_mps2;
std::vector<float> vector_nav_imu_accel_z_mps2;
std::vector<float> vector_nav_imu_gyro_x_radps;
std::vector<float> vector_nav_imu_gyro_y_radps;
std::vector<float> vector_nav_imu_gyro_z_radps;
std::vector<float> vector_nav_mag_x_ut;
std::vector<float> vector_nav_mag_y_ut;
std::vector<float> vector_nav_mag_z_ut;
std::vector<float> vector_nav_static_pres_pa;
std::vector<uint8_t> vector_nav_gnss_installed;
std::vector<uint8_t> vector_nav_gnss_healthy;
std::vector<uint8_t> vector_nav_gnss_new_data;
std::vector<uint8_t> vector_nav_gnss_fix;
std::vector<uint8_t> vector_nav_gnss_num_sats;
std::vector<int16_t> vector_nav_gnss_gps_week;
std::vector<float> vector_nav_gnss_alt_wgs84;
std::vector<float> vector_nav_gnss_horz_acc_m;
std::vector<float> vector_nav_gnss_vert_acc_m;
std::vector<float> vector_nav_gnss_vel_acc_mps;
std::vector<float> vector_nav_gnss_north_vel_mps;
std::vector<float> vector_nav_gnss_east_vel_mps;
std::vector<float> vector_nav_gnss_down_vel_mps;
std::vector<double> vector_nav_gnss_lat_rad;
std::vector<double> vector_nav_gnss_lon_rad;
std::vector<double> vector_nav_gnss_gps_tow_s;
#endif
std::vector<uint8_t> ext_mag_installed;
std::vector<uint8_t> ext_mag_healthy;
std::vector<uint8_t> ext_mag_new_data;
std::vector<float> ext_mag_die_temp_c;
std::vector<float> ext_mag_x_ut;
std::vector<float> ext_mag_y_ut;
std::vector<float> ext_mag_z_ut;
std::vector<uint8_t> ext_gnss1_installed;
std::vector<uint8_t> ext_gnss1_healthy;
std::vector<uint8_t> ext_gnss1_new_data;
std::vector<uint8_t> ext_gnss1_rel_pos_avail;
std::vector<uint8_t> ext_gnss1_rel_pos_moving_baseline;
std::vector<uint8_t> ext_gnss1_rel_pos_baseline_normalized;
std::vector<uint8_t> ext_gnss1_fix;
std::vector<uint8_t> ext_gnss1_num_sats;
std::vector<int16_t> ext_gnss1_gps_week;
std::vector<float> ext_gnss1_alt_wgs84_m;
std::vector<float> ext_gnss1_horz_acc_m;
std::vector<float> ext_gnss1_vert_acc_m;
std::vector<float> ext_gnss1_vel_acc_mps;
std::vector<float> ext_gnss1_north_vel_mps;
std::vector<float> ext_gnss1_east_vel_mps;
std::vector<float> ext_gnss1_down_vel_mps;
std::vector<float> ext_gnss1_rel_pos_acc_north_m;
std::vector<float> ext_gnss1_rel_pos_acc_east_m;
std::vector<float> ext_gnss1_rel_pos_acc_down_m;
std::vector<double> ext_gnss1_gps_tow_s;
std::vector<double> ext_gnss1_lat_rad;
std::vector<double> ext_gnss1_lon_rad;
std::vector<double> ext_gnss1_rel_pos_north_m;
std::vector<double> ext_gnss1_rel_pos_east_m;
std::vector<double> ext_gnss1_rel_pos_down_m;
#if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
    defined(__FMU_R_MINI_V1__)
std::vector<uint8_t> ext_gnss2_installed;
std::vector<uint8_t> ext_gnss2_healthy;
std::vector<uint8_t> ext_gnss2_new_data;
std::vector<uint8_t> ext_gnss2_rel_pos_avail;
std::vector<uint8_t> ext_gnss2_rel_pos_moving_baseline;
std::vector<uint8_t> ext_gnss2_rel_pos_baseline_normalized;
std::vector<uint8_t> ext_gnss2_fix;
std::vector<uint8_t> ext_gnss2_num_sats;
std::vector<int16_t> ext_gnss2_gps_week;
std::vector<float> ext_gnss2_alt_wgs84_m;
std::vector<float> ext_gnss2_horz_acc_m;
std::vector<float> ext_gnss2_vert_acc_m;
std::vector<float> ext_gnss2_vel_acc_mps;
std::vector<float> ext_gnss2_north_vel_mps;
std::vector<float> ext_gnss2_east_vel_mps;
std::vector<float> ext_gnss2_down_vel_mps;
std::vector<float> ext_gnss2_rel_pos_acc_north_m;
std::vector<float> ext_gnss2_rel_pos_acc_east_m;
std::vector<float> ext_gnss2_rel_pos_acc_down_m;
std::vector<double> ext_gnss2_gps_tow_s;
std::vector<double> ext_gnss2_lat_rad;
std::vector<double> ext_gnss2_lon_rad;
std::vector<double> ext_gnss2_rel_pos_north_m;
std::vector<double> ext_gnss2_rel_pos_east_m;
std::vector<double> ext_gnss2_rel_pos_down_m;
#endif
std::vector<uint8_t> ext_pres1_installed;
std::vector<uint8_t> ext_pres1_is_static_pres;
std::vector<uint8_t> ext_pres1_healthy;
std::vector<uint8_t> ext_pres1_new_data;
std::vector<float> ext_pres1_die_temp_c;
std::vector<float> ext_pres1_pres_pa;
std::vector<uint8_t> ext_pres2_installed;
std::vector<uint8_t> ext_pres2_is_static_pres;
std::vector<uint8_t> ext_pres2_healthy;
std::vector<uint8_t> ext_pres2_new_data;
std::vector<float> ext_pres2_die_temp_c;
std::vector<float> ext_pres2_pres_pa;
std::vector<uint8_t> ext_pres3_installed;
std::vector<uint8_t> ext_pres3_is_static_pres;
std::vector<uint8_t> ext_pres3_healthy;
std::vector<uint8_t> ext_pres3_new_data;
std::vector<float> ext_pres3_die_temp_c;
std::vector<float> ext_pres3_pres_pa;
std::vector<uint8_t> ext_pres4_installed;
std::vector<uint8_t> ext_pres4_is_static_pres;
std::vector<uint8_t> ext_pres4_healthy;
std::vector<uint8_t> ext_pres4_new_data;
std::vector<float> ext_pres4_die_temp_c;
std::vector<float> ext_pres4_pres_pa;
#if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
    defined(__FMU_R_MINI_V1__)
std::vector<uint8_t> rad_alt_installed;
std::vector<uint8_t> rad_alt_healthy;
std::vector<uint8_t> rad_alt_new_data;
std::vector<uint8_t> rad_alt_snr;
std::vector<float> rad_alt_alt_m;
std::vector<uint8_t> opflow_installed;
std::vector<uint8_t> opflow_healthy;
std::vector<uint8_t> opflow_new_data;
std::vector<uint8_t> opflow_sur_qual;
std::vector<uint8_t> opflow_range_qual;
std::vector<float> opflow_mot_x;
std::vector<float> opflow_mot_y;
std::vector<float> opflow_range_mm;
#endif
std::vector<float> ain0_v;
std::vector<float> ain1_v;
#if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
    defined(__FMU_R_MINI_V1__)
std::vector<float> ain2_v;
std::vector<float> ain3_v;
std::vector<float> ain4_v;
std::vector<float> ain5_v;
#endif
#if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__)
std::vector<float> ain6_v;
std::vector<float> ain7_v;
#endif
#if defined(__FMU_R_V2__) || defined(__FMU_R_MINI_V1__)
std::vector<float> power_module_voltage_v;
std::vector<float> power_module_current_ma;
#endif
std::vector<uint8_t> bfs_ins_initialized;
std::vector<float> bfs_ins_pitch_rad;
std::vector<float> bfs_ins_roll_rad;
std::vector<float> bfs_ins_heading_rad;
std::vector<float> bfs_ins_alt_wgs84_m;
std::vector<float> bfs_ins_accel_x_mps2;
std::vector<float> bfs_ins_accel_y_mps2;
std::vector<float> bfs_ins_accel_z_mps2;
std::vector<float> bfs_ins_gyro_x_radps;
std::vector<float> bfs_ins_gyro_y_radps;
std::vector<float> bfs_ins_gyro_z_radps;
std::vector<float> bfs_ins_mag_x_ut;
std::vector<float> bfs_ins_mag_y_ut;
std::vector<float> bfs_ins_mag_z_ut;
std::vector<float> bfs_ins_north_vel_mps;
std::vector<float> bfs_ins_east_vel_mps;
std::vector<float> bfs_ins_down_vel_mps;
std::vector<double> bfs_ins_lat_rad;
std::vector<double> bfs_ins_lon_rad;
#if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
    defined(__FMU_R_V2_BETA__)
std::vector<uint8_t> vector_nav_ins_initialized;
std::vector<float> vector_nav_ins_pitch_rad;
std::vector<float> vector_nav_ins_roll_rad;
std::vector<float> vector_nav_ins_heading_rad;
std::vector<float> vector_nav_ins_alt_wgs84_m;
std::vector<float> vector_nav_ins_accel_x_mps2;
std::vector<float> vector_nav_ins_accel_y_mps2;
std::vector<float> vector_nav_ins_accel_z_mps2;
std::vector<float> vector_nav_ins_gyro_x_radps;
std::vector<float> vector_nav_ins_gyro_y_radps;
std::vector<float> vector_nav_ins_gyro_z_radps;
std::vector<float> vector_nav_ins_mag_x_ut;
std::vector<float> vector_nav_ins_mag_y_ut;
std::vector<float> vector_nav_ins_mag_z_ut;
std::vector<float> vector_nav_ins_north_vel_mps;
std::vector<float> vector_nav_ins_east_vel_mps;
std::vector<float> vector_nav_ins_down_vel_mps;
std::vector<double> vector_nav_ins_lat_rad;
std::vector<double> vector_nav_ins_lon_rad;
#endif
std::vector<float> aux_ins_home_alt_wgs84_m;
std::vector<float> aux_ins_gnd_spd_mps;
std::vector<float> aux_ins_gnd_track_rad;
std::vector<float> aux_ins_flight_path_rad;
std::vector<double> aux_ins_home_lat_rad;
std::vector<double> aux_ins_home_lon_rad;
std::vector<double> aux_ins_ned_pos_north_m;
std::vector<double> aux_ins_ned_pos_east_m;
std::vector<double> aux_ins_ned_pos_down_m;
std::vector<float> adc_static_pres_pa;
std::vector<float> adc_diff_pres_pa;
std::vector<float> adc_pres_alt_m;
std::vector<float> adc_rel_alt_m;
std::vector<float> adc_ias_mps;
std::vector<uint8_t> telem_waypoint_frame;
std::vector<uint16_t> telem_waypoint_cmd;
std::vector<float> telem_waypoint_param1;
std::vector<float> telem_waypoint_param2;
std::vector<float> telem_waypoint_param3;
std::vector<float> telem_waypoint_param4;
std::vector<int32_t> telem_waypoint_x;
std::vector<int32_t> telem_waypoint_y;
std::vector<float> telem_waypoint_z;
std::vector<float> telem_param0;
std::vector<float> telem_param1;
std::vector<float> telem_param2;
std::vector<float> telem_param3;
std::vector<float> telem_param4;
std::vector<float> telem_param5;
std::vector<float> telem_param6;
std::vector<float> telem_param7;
std::vector<float> telem_param8;
std::vector<float> telem_param9;
std::vector<float> telem_param10;
std::vector<float> telem_param11;
std::vector<float> telem_param12;
std::vector<float> telem_param13;
std::vector<float> telem_param14;
std::vector<float> telem_param15;
std::vector<float> telem_param16;
std::vector<float> telem_param17;
std::vector<float> telem_param18;
std::vector<float> telem_param19;
std::vector<float> telem_param20;
std::vector<float> telem_param21;
std::vector<float> telem_param22;
std::vector<float> telem_param23;
std::vector<uint8_t> vms_advance_waypoint;
std::vector<uint8_t> vms_motors_enabled;
std::vector<uint8_t> vms_mode;
std::vector<int16_t> vms_sbus_cmd0;
std::vector<int16_t> vms_sbus_cmd1;
std::vector<int16_t> vms_sbus_cmd2;
std::vector<int16_t> vms_sbus_cmd3;
std::vector<int16_t> vms_sbus_cmd4;
std::vector<int16_t> vms_sbus_cmd5;
std::vector<int16_t> vms_sbus_cmd6;
std::vector<int16_t> vms_sbus_cmd7;
std::vector<int16_t> vms_sbus_cmd8;
std::vector<int16_t> vms_sbus_cmd9;
std::vector<int16_t> vms_sbus_cmd10;
std::vector<int16_t> vms_sbus_cmd11;
std::vector<int16_t> vms_sbus_cmd12;
std::vector<int16_t> vms_sbus_cmd13;
std::vector<int16_t> vms_sbus_cmd14;
std::vector<int16_t> vms_sbus_cmd15;
std::vector<int16_t> vms_pwm_cmd0;
std::vector<int16_t> vms_pwm_cmd1;
std::vector<int16_t> vms_pwm_cmd2;
std::vector<int16_t> vms_pwm_cmd3;
std::vector<int16_t> vms_pwm_cmd4;
std::vector<int16_t> vms_pwm_cmd5;
std::vector<int16_t> vms_pwm_cmd6;
std::vector<int16_t> vms_pwm_cmd7;
std::vector<float> vms_flight_time_remaining_s;
std::vector<float> vms_power_remaining_prcnt;
std::vector<float> vms_throttle_cmd_prcnt;
std::vector<float> vms_aux0;
std::vector<float> vms_aux1;
std::vector<float> vms_aux2;
std::vector<float> vms_aux3;
std::vector<float> vms_aux4;
std::vector<float> vms_aux5;
std::vector<float> vms_aux6;
std::vector<float> vms_aux7;
std::vector<float> vms_aux8;
std::vector<float> vms_aux9;
std::vector<float> vms_aux10;
std::vector<float> vms_aux11;
std::vector<float> vms_aux12;
std::vector<float> vms_aux13;
std::vector<float> vms_aux14;
std::vector<float> vms_aux15;
std::vector<float> vms_aux16;
std::vector<float> vms_aux17;
std::vector<float> vms_aux18;
std::vector<float> vms_aux19;
std::vector<float> vms_aux20;
std::vector<float> vms_aux21;
std::vector<float> vms_aux22;
std::vector<float> vms_aux23;

template<typename T>
T Scale(uint64_t val, T min, T max, T sf, T bias) {
  static_assert(std::is_floating_point<T>::value,
                "Only floating point types supported");
  T ret = static_cast<T>(val) / sf - bias / sf;
  if (ret > max) {ret = max;}
  if (ret < min) {ret = min;}
  return ret;
}

DatalogMsg datalog_msg_;

uint32_t temp_;
float tempf_;
int32_t tempi_;

int main(int argc, char** argv) {
  /* Input file name */
  std::string input_file_name(argv[1]);
  /* Check input file extension */
  std::string bfs_ext = ".bfs";
  std::cout << "Parsing file " << input_file_name << "...";
  if (input_file_name.compare(input_file_name.length() - bfs_ext.length(), bfs_ext.length(), bfs_ext) != 0) {
    std::cerr << "ERROR: Input file must be a BFS log file, which has a .bfs extension." << std::endl;
    return -1;
  }
  /* Try to read the flight data */
  FILE *input = fopen(input_file_name.c_str(), "rb");
  if (!input) {
    std::cerr << "ERROR: Unable to open input file, maybe the path is incorrect." << std::endl;
    return -1;
  }
  /* Create the output file */
  std::string mat_ext = ".mat";
  std::string output_file_name = input_file_name;
  output_file_name.replace(output_file_name.length() - bfs_ext.length(), bfs_ext.length(), mat_ext);
  FILE *output = fopen(output_file_name.c_str(), "wb");
  if (!output) {
    std::cerr << "ERROR: Unable to open output file." << std::endl;
    return -1;
  }
  /* Read file in chunks */
  static constexpr int CHUNK_SIZE = 1024;
  uint8_t buffer[CHUNK_SIZE];
  std::size_t bytes_read = 0;
  /* Framing */
  bfs::FrameDecoder<CHUNK_SIZE> temp_decoder;
  /* Iterate through the file and populate vectors */
  std::size_t num_packets = 0;
  while ((bytes_read = fread(buffer, 1, sizeof(buffer), input)) > 0) {
    for (std::size_t i = 0; i < bytes_read; i++) {
      if (temp_decoder.Found(buffer[i])) {
        memcpy(&datalog_msg_, temp_decoder.data(), sizeof(datalog_msg_));
        sys_frame_time_s.push_back(static_cast<double>(datalog_msg_.sys_frame_time_us) / 1e6);
        #if defined(__FMU_R_V1__)
        sys_input_volt.push_back(Scale(datalog_msg_.sys_input_volt, 0.0f, 36.0f, 113.75f, 0.0f));
        sys_reg_volt.push_back(Scale(datalog_msg_.sys_reg_volt, 2.5f, 6.5f, 127.75f, -319.375f));
        sys_pwm_volt.push_back(Scale(datalog_msg_.sys_pwm_volt, 0.0f, 10.0f, 102.3f, 0.0f));
        sys_sbus_volt.push_back(Scale(datalog_msg_.sys_sbus_volt, 0.0f, 10.0f, 102.3f, 0.0f));
        #endif
        sys_time_s.push_back(static_cast<double>(datalog_msg_.sys_time_us) / 1e6);
        incept_new_data.push_back(datalog_msg_.incept_new_data);
        incept_lost_frame.push_back(datalog_msg_.incept_lost_frame);
        incept_failsafe.push_back(datalog_msg_.incept_failsafe);
        incept_ch0.push_back(datalog_msg_.incept_ch0);
        incept_ch1.push_back(datalog_msg_.incept_ch1);
        incept_ch2.push_back(datalog_msg_.incept_ch2);
        incept_ch3.push_back(datalog_msg_.incept_ch3);
        incept_ch4.push_back(datalog_msg_.incept_ch4);
        incept_ch5.push_back(datalog_msg_.incept_ch5);
        incept_ch6.push_back(datalog_msg_.incept_ch6);
        incept_ch7.push_back(datalog_msg_.incept_ch7);
        incept_ch8.push_back(datalog_msg_.incept_ch8);
        incept_ch9.push_back(datalog_msg_.incept_ch9);
        incept_ch10.push_back(datalog_msg_.incept_ch10);
        incept_ch11.push_back(datalog_msg_.incept_ch11);
        incept_ch12.push_back(datalog_msg_.incept_ch12);
        incept_ch13.push_back(datalog_msg_.incept_ch13);
        incept_ch14.push_back(datalog_msg_.incept_ch14);
        incept_ch15.push_back(datalog_msg_.incept_ch15);
        fmu_imu_healthy.push_back(datalog_msg_.fmu_imu_healthy);
        fmu_imu_new_data.push_back(datalog_msg_.fmu_imu_new_data);
        fmu_imu_die_temp_c.push_back(Scale(datalog_msg_.fmu_imu_die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f));
        fmu_imu_accel_x_mps2.push_back(Scale(datalog_msg_.fmu_imu_accel_x_mps2, -156.9064f, 156.9064f, 208.834693804714f, 32767.5f));
        fmu_imu_accel_y_mps2.push_back(Scale(datalog_msg_.fmu_imu_accel_y_mps2, -156.9064f, 156.9064f, 208.834693804714f, 32767.5f));
        fmu_imu_accel_z_mps2.push_back(Scale(datalog_msg_.fmu_imu_accel_z_mps2, -156.9064f, 156.9064f, 208.834693804714f, 32767.5f));
        fmu_imu_gyro_x_radps.push_back(Scale(datalog_msg_.fmu_imu_gyro_x_radps, -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f));
        fmu_imu_gyro_y_radps.push_back(Scale(datalog_msg_.fmu_imu_gyro_y_radps, -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f));
        fmu_imu_gyro_z_radps.push_back(Scale(datalog_msg_.fmu_imu_gyro_z_radps, -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f));
        fmu_mag_healthy.push_back(datalog_msg_.fmu_mag_healthy );
        fmu_mag_new_data.push_back(datalog_msg_.fmu_mag_new_data);
        fmu_mag_die_temp_c.push_back(Scale(datalog_msg_.fmu_mag_die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f));
        fmu_mag_x_ut.push_back(Scale(datalog_msg_.fmu_mag_x_ut, -250.0f, 250.0f, 131.07f, 32767.5f));
        fmu_mag_y_ut.push_back(Scale(datalog_msg_.fmu_mag_y_ut, -250.0f, 250.0f, 131.07f, 32767.5f));
        fmu_mag_z_ut.push_back(Scale(datalog_msg_.fmu_mag_z_ut, -250.0f, 250.0f, 131.07f, 32767.5f));
        fmu_static_pres_healthy.push_back(datalog_msg_.fmu_static_pres_healthy);
        fmu_static_pres_new_data.push_back(datalog_msg_.fmu_static_pres_new_data);
        fmu_static_pres_die_temp_c.push_back(Scale(datalog_msg_.fmu_static_pres_die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f));
        fmu_static_pres_pa.push_back(Scale(datalog_msg_.fmu_static_pres_pa, 60000.0f, 120000.0f, 1.09225f, -65535.0f));
        #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
            defined(__FMU_R_V2_BETA__)
        vector_nav_installed.push_back(datalog_msg_.vector_nav_installed);
        vector_nav_healthy.push_back(datalog_msg_.vector_nav_healthy);
        vector_nav_die_temp_c.push_back(Scale(datalog_msg_.vector_nav_die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f));
        vector_nav_imu_new_data.push_back(datalog_msg_.vector_nav_imu_new_data);
        vector_nav_imu_accel_x_mps2.push_back(Scale(datalog_msg_.vector_nav_imu_accel_x_mps2, -156.9064f, 156.9064f, 208.834693804714f, 32767.5f));
        vector_nav_imu_accel_y_mps2.push_back(Scale(datalog_msg_.vector_nav_imu_accel_y_mps2, -156.9064f, 156.9064f, 208.834693804714f, 32767.5f));
        vector_nav_imu_accel_z_mps2.push_back(Scale(datalog_msg_.vector_nav_imu_accel_z_mps2, -156.9064f, 156.9064f, 208.834693804714f, 32767.5f));
        vector_nav_imu_gyro_x_radps.push_back(Scale(datalog_msg_.vector_nav_imu_gyro_x_radps, -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f));
        vector_nav_imu_gyro_y_radps.push_back(Scale(datalog_msg_.vector_nav_imu_gyro_y_radps, -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f));
        vector_nav_imu_gyro_z_radps.push_back(Scale(datalog_msg_.vector_nav_imu_gyro_z_radps, -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f));
        vector_nav_mag_x_ut.push_back(Scale(datalog_msg_.vector_nav_mag_x_ut, -250.0f, 250.0f, 131.07f, 32767.5f));
        vector_nav_mag_y_ut.push_back(Scale(datalog_msg_.vector_nav_mag_y_ut, -250.0f, 250.0f, 131.07f, 32767.5f));
        vector_nav_mag_z_ut.push_back(Scale(datalog_msg_.vector_nav_mag_z_ut, -250.0f, 250.0f, 131.07f, 32767.5f));
        vector_nav_static_pres_pa.push_back(Scale(datalog_msg_.vector_nav_static_pres_pa, 60000.0f, 120000.0f, 1.09225f, -65535.0f));
        vector_nav_gnss_healthy.push_back(datalog_msg_.vector_nav_gnss_healthy);
        vector_nav_gnss_new_data.push_back(datalog_msg_.vector_nav_gnss_new_data);
        vector_nav_gnss_fix.push_back(datalog_msg_.vector_nav_gnss_fix);
        vector_nav_gnss_num_sats.push_back(datalog_msg_.vector_nav_gnss_num_sats);
        vector_nav_gnss_gps_week.push_back(datalog_msg_.vector_nav_gnss_gps_week);
        vector_nav_gnss_alt_wgs84.push_back(Scale(datalog_msg_.vector_nav_gnss_alt_wgs84, -500.0f, 5000.0f, 11.9154545454545f, 5957.72727272727f));
        vector_nav_gnss_horz_acc_m.push_back(Scale(datalog_msg_.vector_nav_gnss_horz_acc_m, 0.0f, 10.0f, 1638.3f, 0.0f));
        vector_nav_gnss_vert_acc_m.push_back(Scale(datalog_msg_.vector_nav_gnss_vert_acc_m, 0.0f, 10.0f, 1638.3f, 0.0f));
        vector_nav_gnss_vel_acc_mps.push_back(Scale(datalog_msg_.vector_nav_gnss_vel_acc_mps, 0.0f, 10.0f, 1638.3f, 0.0f));
        vector_nav_gnss_north_vel_mps.push_back(Scale(datalog_msg_.vector_nav_gnss_north_vel_mps, -60.0f, 60.0f, 17.0583333333333f, 1023.5f));
        vector_nav_gnss_east_vel_mps.push_back(Scale(datalog_msg_.vector_nav_gnss_east_vel_mps, -60.0f, 60.0f, 17.0583333333333f, 1023.5f));
        vector_nav_gnss_down_vel_mps.push_back(Scale(datalog_msg_.vector_nav_gnss_down_vel_mps, -60.0f, 60.0f, 17.0583333333333f, 1023.5f));
        vector_nav_gnss_lat_rad.push_back(Scale(datalog_msg_.vector_nav_gnss_lat_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 11930464.7083333, 2147483647.5));
        vector_nav_gnss_lon_rad.push_back(Scale(datalog_msg_.vector_nav_gnss_lon_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 11930464.7083333, 2147483647.5));
        vector_nav_gnss_gps_tow_s.push_back(Scale(datalog_msg_.vector_nav_gnss_gps_tow_s, 0.0, 604800.0, 1775.36677083333, 0.0));
        #endif
        ext_mag_installed.push_back(datalog_msg_.ext_mag_installed);
        ext_mag_healthy.push_back(datalog_msg_.ext_mag_healthy);
        ext_mag_new_data.push_back(datalog_msg_.ext_mag_new_data);
        ext_mag_die_temp_c.push_back(Scale(datalog_msg_.ext_mag_die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f));
        ext_mag_x_ut.push_back(Scale(datalog_msg_.ext_mag_x_ut, -250.0f, 250.0f, 131.07f, 32767.5f));
        ext_mag_y_ut.push_back(Scale(datalog_msg_.ext_mag_y_ut, -250.0f, 250.0f, 131.07f, 32767.5f));
        ext_mag_z_ut.push_back(Scale(datalog_msg_.ext_mag_z_ut, -250.0f, 250.0f, 131.07f, 32767.5f));
        ext_gnss1_installed.push_back(datalog_msg_.ext_gnss1_installed);
        ext_gnss1_healthy.push_back(datalog_msg_.ext_gnss1_healthy);
        ext_gnss1_new_data.push_back(datalog_msg_.ext_gnss1_new_data);
        ext_gnss1_rel_pos_avail.push_back(datalog_msg_.ext_gnss1_rel_pos_avail);
        ext_gnss1_rel_pos_moving_baseline.push_back(datalog_msg_.ext_gnss1_rel_pos_moving_baseline);
        ext_gnss1_rel_pos_baseline_normalized.push_back(datalog_msg_.ext_gnss1_rel_pos_baseline_normalized);
        ext_gnss1_fix.push_back(datalog_msg_.ext_gnss1_fix);
        ext_gnss1_num_sats.push_back(datalog_msg_.ext_gnss1_num_sats);
        ext_gnss1_gps_week.push_back(datalog_msg_.ext_gnss1_gps_week);
        ext_gnss1_alt_wgs84_m.push_back(Scale(datalog_msg_.ext_gnss1_alt_wgs84_m, -500.0f, 5000.0f, 190.65f, 95324.9999999999f));
        ext_gnss1_horz_acc_m.push_back(Scale(datalog_msg_.ext_gnss1_horz_acc_m, 0.0f, 10.0f, 1638.3f, 0.0f));
        ext_gnss1_vert_acc_m.push_back(Scale(datalog_msg_.ext_gnss1_vert_acc_m, 0.0f, 10.0f, 1638.3f, 0.0f));
        ext_gnss1_vel_acc_mps.push_back(Scale(datalog_msg_.ext_gnss1_vel_acc_mps, 0.0f, 10.0f, 1638.3f, 0.0f));
        ext_gnss1_north_vel_mps.push_back(Scale(datalog_msg_.ext_gnss1_north_vel_mps, -60.0f, 60.0f, 136.525f, 8191.5f));
        ext_gnss1_east_vel_mps.push_back(Scale(datalog_msg_.ext_gnss1_east_vel_mps, -60.0f, 60.0f, 136.525f, 8191.5f));
        ext_gnss1_down_vel_mps.push_back(Scale(datalog_msg_.ext_gnss1_down_vel_mps, -60.0f, 60.0f, 136.525f, 8191.5f));
        ext_gnss1_lat_rad.push_back(Scale(datalog_msg_.ext_gnss1_lat_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5));
        ext_gnss1_lon_rad.push_back(Scale(datalog_msg_.ext_gnss1_lon_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5));
        ext_gnss1_gps_tow_s.push_back(Scale(datalog_msg_.ext_gnss1_gps_tow_s, 0.0, 604800.0, 1775.36677083333, 0.0));
        ext_gnss1_rel_pos_acc_north_m.push_back(Scale(datalog_msg_.ext_gnss1_rel_pos_acc_north_m, 0.0f, 10.0f, 6553.5f, 0.0f));
        ext_gnss1_rel_pos_acc_east_m.push_back(Scale(datalog_msg_.ext_gnss1_rel_pos_acc_east_m, 0.0f, 10.0f, 6553.5f, 0.0f));
        ext_gnss1_rel_pos_acc_down_m.push_back(Scale(datalog_msg_.ext_gnss1_rel_pos_acc_down_m, 0.0f, 10.0f, 6553.5f, 0.0f));
        ext_gnss1_rel_pos_north_m.push_back(Scale(datalog_msg_.ext_gnss1_rel_pos_north_m, -50000.0, 50000.0, 1342.17727, 67108863.5));
        ext_gnss1_rel_pos_east_m.push_back(Scale(datalog_msg_.ext_gnss1_rel_pos_east_m, -50000.0, 50000.0, 1342.17727, 67108863.5));
        ext_gnss1_rel_pos_down_m.push_back(Scale(datalog_msg_.ext_gnss1_rel_pos_down_m, -5000.0, 500.0, 1525.20127272727, 7626006.36363636));
        #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
            defined(__FMU_R_MINI_V1__)
        ext_gnss2_installed.push_back(datalog_msg_.ext_gnss2_installed);
        ext_gnss2_healthy.push_back(datalog_msg_.ext_gnss2_healthy);
        ext_gnss2_new_data.push_back(datalog_msg_.ext_gnss2_new_data);
        ext_gnss2_rel_pos_avail.push_back(datalog_msg_.ext_gnss2_rel_pos_avail);
        ext_gnss2_rel_pos_moving_baseline.push_back(datalog_msg_.ext_gnss2_rel_pos_moving_baseline);
        ext_gnss2_rel_pos_baseline_normalized.push_back(datalog_msg_.ext_gnss2_rel_pos_baseline_normalized);
        ext_gnss2_fix.push_back(datalog_msg_.ext_gnss2_fix);
        ext_gnss2_num_sats.push_back(datalog_msg_.ext_gnss2_num_sats);
        ext_gnss2_gps_week.push_back(datalog_msg_.ext_gnss2_gps_week);
        ext_gnss2_alt_wgs84_m.push_back(Scale(datalog_msg_.ext_gnss2_alt_wgs84_m, -500.0f, 5000.0f, 190.65f, 95324.9999999999f));
        ext_gnss2_horz_acc_m.push_back(Scale(datalog_msg_.ext_gnss2_horz_acc_m, 0.0f, 10.0f, 1638.3f, 0.0f));
        ext_gnss2_vert_acc_m.push_back(Scale(datalog_msg_.ext_gnss2_vert_acc_m, 0.0f, 10.0f, 1638.3f, 0.0f));
        ext_gnss2_vel_acc_mps.push_back(Scale(datalog_msg_.ext_gnss2_vel_acc_mps, 0.0f, 10.0f, 1638.3f, 0.0f));
        ext_gnss2_north_vel_mps.push_back(Scale(datalog_msg_.ext_gnss2_north_vel_mps, -60.0f, 60.0f, 136.525f, 8191.5f));
        ext_gnss2_east_vel_mps.push_back(Scale(datalog_msg_.ext_gnss2_east_vel_mps, -60.0f, 60.0f, 136.525f, 8191.5f));
        ext_gnss2_down_vel_mps.push_back(Scale(datalog_msg_.ext_gnss2_down_vel_mps, -60.0f, 60.0f, 136.525f, 8191.5f));
        ext_gnss2_lat_rad.push_back(Scale(datalog_msg_.ext_gnss2_lat_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5));
        ext_gnss2_lon_rad.push_back(Scale(datalog_msg_.ext_gnss2_lon_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5));
        ext_gnss2_gps_tow_s.push_back(Scale(datalog_msg_.ext_gnss2_gps_tow_s, 0.0, 604800.0, 1775.36677083333, 0.0));
        ext_gnss2_rel_pos_acc_north_m.push_back(Scale(datalog_msg_.ext_gnss2_rel_pos_acc_north_m, 0.0f, 10.0f, 6553.5f, 0.0f));
        ext_gnss2_rel_pos_acc_east_m.push_back(Scale(datalog_msg_.ext_gnss2_rel_pos_acc_east_m, 0.0f, 10.0f, 6553.5f, 0.0f));
        ext_gnss2_rel_pos_acc_down_m.push_back(Scale(datalog_msg_.ext_gnss2_rel_pos_acc_down_m, 0.0f, 10.0f, 6553.5f, 0.0f));
        ext_gnss2_rel_pos_north_m.push_back(Scale(datalog_msg_.ext_gnss2_rel_pos_north_m, -50000.0, 50000.0, 1342.17727, 67108863.5));
        ext_gnss2_rel_pos_east_m.push_back(Scale(datalog_msg_.ext_gnss2_rel_pos_east_m, -50000.0, 50000.0, 1342.17727, 67108863.5));
        ext_gnss2_rel_pos_down_m.push_back(Scale(datalog_msg_.ext_gnss2_rel_pos_down_m, -5000.0, 500.0, 1525.20127272727, 7626006.36363636));
        #endif
        ext_pres1_installed.push_back(datalog_msg_.ext_pres1_installed);
        ext_pres1_is_static_pres.push_back(datalog_msg_.ext_pres1_is_static_pres);
        ext_pres1_healthy.push_back(datalog_msg_.ext_pres1_healthy);
        ext_pres1_new_data.push_back(datalog_msg_.ext_pres1_new_data);
        ext_pres1_die_temp_c.push_back(Scale(datalog_msg_.ext_pres1_die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f));
        if (datalog_msg_.ext_pres1_is_static_pres) {
          ext_pres1_pres_pa.push_back(Scale(datalog_msg_.ext_pres1_pres_pa, 60000.0f, 120000.0f, 1.09225f, -65535.0f));
        } else {
          ext_pres1_pres_pa.push_back(Scale(datalog_msg_.ext_pres1_pres_pa, -2000.0f, 2000.0f, 16.38375f, 32767.5f));
        }
        ext_pres2_installed.push_back(datalog_msg_.ext_pres2_installed);
        ext_pres2_is_static_pres.push_back(datalog_msg_.ext_pres2_is_static_pres);
        ext_pres2_healthy.push_back(datalog_msg_.ext_pres2_healthy);
        ext_pres2_new_data.push_back(datalog_msg_.ext_pres2_new_data);
        ext_pres2_die_temp_c.push_back(Scale(datalog_msg_.ext_pres2_die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f));
        if (datalog_msg_.ext_pres2_is_static_pres) {
          ext_pres2_pres_pa.push_back(Scale(datalog_msg_.ext_pres2_pres_pa, 60000.0f, 120000.0f, 1.09225f, -65535.0f));
        } else {
          ext_pres2_pres_pa.push_back(Scale(datalog_msg_.ext_pres2_pres_pa, -2000.0f, 2000.0f, 16.38375f, 32767.5f));
        }
        ext_pres3_installed.push_back(datalog_msg_.ext_pres3_installed);
        ext_pres3_is_static_pres.push_back(datalog_msg_.ext_pres3_is_static_pres);
        ext_pres3_healthy.push_back(datalog_msg_.ext_pres3_healthy);
        ext_pres3_new_data.push_back(datalog_msg_.ext_pres3_new_data);
        ext_pres3_die_temp_c.push_back(Scale(datalog_msg_.ext_pres3_die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f));
        if (datalog_msg_.ext_pres3_is_static_pres) {
          ext_pres3_pres_pa.push_back(Scale(datalog_msg_.ext_pres3_pres_pa, 60000.0f, 120000.0f, 1.09225f, -65535.0f));
        } else {
          ext_pres3_pres_pa.push_back(Scale(datalog_msg_.ext_pres3_pres_pa, -2000.0f, 2000.0f, 16.38375f, 32767.5f));
        }
        ext_pres4_installed.push_back(datalog_msg_.ext_pres4_installed);
        ext_pres4_is_static_pres.push_back(datalog_msg_.ext_pres4_is_static_pres);
        ext_pres4_healthy.push_back(datalog_msg_.ext_pres4_healthy);
        ext_pres4_new_data.push_back(datalog_msg_.ext_pres4_new_data);
        ext_pres4_die_temp_c.push_back(Scale(datalog_msg_.ext_pres4_die_temp_c, -40.0f, 80.0f, 8.525f, 341.0f));
        if (datalog_msg_.ext_pres4_is_static_pres) {
          ext_pres4_pres_pa.push_back(Scale(datalog_msg_.ext_pres4_pres_pa, 60000.0f, 120000.0f, 1.09225f, -65535.0f));
        } else {
          ext_pres4_pres_pa.push_back(Scale(datalog_msg_.ext_pres4_pres_pa, -2000.0f, 2000.0f, 16.38375f, 32767.5f));
        }
        #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
            defined(__FMU_R_MINI_V1__)
        rad_alt_installed.push_back(datalog_msg_.rad_alt_installed);
        rad_alt_healthy.push_back(datalog_msg_.rad_alt_healthy);
        rad_alt_new_data.push_back(datalog_msg_.rad_alt_new_data);
        rad_alt_snr.push_back(datalog_msg_.rad_alt_snr);
        rad_alt_alt_m.push_back(Scale(datalog_msg_.rad_alt_alt_m, 0.0f, 50.0f, 40.94f, 0.0f));
        opflow_installed.push_back(datalog_msg_.opflow_installed);
        opflow_healthy.push_back(datalog_msg_.opflow_healthy);
        opflow_new_data.push_back(datalog_msg_.opflow_new_data);
        opflow_sur_qual.push_back(datalog_msg_.opflow_sur_qual);
        opflow_range_qual.push_back(datalog_msg_.opflow_range_qual);
        opflow_mot_x.push_back(Scale(datalog_msg_.opflow_mot_x, -500.0f, 500.0f, 1.0f, 600.0f));
        opflow_mot_y.push_back(Scale(datalog_msg_.opflow_mot_y, -500.0f, 500.0f, 1.0f, 600.0f));
        opflow_range_mm.push_back(Scale(datalog_msg_.opflow_range_mm, -1.0f, 2000.0f, 1.0f, 3000.0f));
        #endif
        ain0_v.push_back(Scale(datalog_msg_.ain0_v, 0.0f, 3.3f, 1240.90909090909f, 0.0f));
        ain1_v.push_back(Scale(datalog_msg_.ain1_v, 0.0f, 3.3f, 1240.90909090909f, 0.0f));
        #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
            defined(__FMU_R_MINI_V1__)
        ain2_v.push_back(Scale(datalog_msg_.ain2_v, 0.0f, 3.3f, 1240.90909090909f, 0.0f));
        ain3_v.push_back(Scale(datalog_msg_.ain3_v, 0.0f, 3.3f, 1240.90909090909f, 0.0f));
        ain4_v.push_back(Scale(datalog_msg_.ain4_v, 0.0f, 3.3f, 1240.90909090909f, 0.0f));
        ain5_v.push_back(Scale(datalog_msg_.ain5_v, 0.0f, 3.3f, 1240.90909090909f, 0.0f));
        #endif
        #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__)
        ain6_v.push_back(Scale(datalog_msg_.ain6_v, 0.0f, 3.3f, 1240.90909090909f, 0.0f));
        ain7_v.push_back(Scale(datalog_msg_.ain7_v, 0.0f, 3.3f, 1240.90909090909f, 0.0f));
        #endif
        #if defined(__FMU_R_V2__) || defined(__FMU_R_MINI_V1__)
        power_module_voltage_v.push_back(Scale(datalog_msg_.power_module_voltage_v, 0.0f, 90.0f, 45.5f, 0.0f));
        power_module_current_ma.push_back(Scale(datalog_msg_.power_module_current_ma, 0.0f, 180000.0f, 1.45635f, 0.0f));
        #endif
        bfs_ins_initialized.push_back(datalog_msg_.bfs_ins_initialized);
        bfs_ins_pitch_rad.push_back(Scale(datalog_msg_.bfs_ins_pitch_rad, -bfs::BFS_PI<float> / 2.0f, bfs::BFS_PI<float> / 2.0f, 20860.4383910547f, 32767.5f));
        bfs_ins_roll_rad.push_back(Scale(datalog_msg_.bfs_ins_roll_rad, -bfs::BFS_PI<float> / 2.0f, bfs::BFS_PI<float> / 2.0f, 20860.4383910547f, 32767.5f));
        bfs_ins_heading_rad.push_back(Scale(datalog_msg_.bfs_ins_heading_rad, -bfs::BFS_PI<float>, bfs::BFS_PI<float>, 10430.2191955274f, 32767.5f));
        bfs_ins_alt_wgs84_m.push_back(Scale(datalog_msg_.bfs_ins_alt_wgs84_m, -500.0f, 5000.0f, 190.65f, 95324.9999999999f));
        bfs_ins_accel_x_mps2.push_back(Scale(datalog_msg_.bfs_ins_accel_x_mps2, -156.9064f, 156.9064f, 208.834693804714f, 32767.5f));
        bfs_ins_accel_y_mps2.push_back(Scale(datalog_msg_.bfs_ins_accel_y_mps2, -156.9064f, 156.9064f, 208.834693804714f, 32767.5f));
        bfs_ins_accel_z_mps2.push_back(Scale(datalog_msg_.bfs_ins_accel_z_mps2, -156.9064f, 156.9064f, 208.834693804714f, 32767.5f));
        bfs_ins_gyro_x_radps.push_back(Scale(datalog_msg_.bfs_ins_gyro_x_radps, -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f));
        bfs_ins_gyro_y_radps.push_back(Scale(datalog_msg_.bfs_ins_gyro_y_radps, -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f));
        bfs_ins_gyro_z_radps.push_back(Scale(datalog_msg_.bfs_ins_gyro_z_radps, -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f));
        bfs_ins_mag_x_ut.push_back(Scale(datalog_msg_.bfs_ins_mag_x_ut, -250.0f, 250.0f, 131.07f, 32767.5f));
        bfs_ins_mag_y_ut.push_back(Scale(datalog_msg_.bfs_ins_mag_y_ut, -250.0f, 250.0f, 131.07f, 32767.5f));
        bfs_ins_mag_z_ut.push_back(Scale(datalog_msg_.bfs_ins_mag_z_ut, -250.0f, 250.0f, 131.07f, 32767.5f));
        bfs_ins_north_vel_mps.push_back(Scale(datalog_msg_.bfs_ins_north_vel_mps, -60.0f, 60.0f, 136.525f, 8191.5f));
        bfs_ins_east_vel_mps.push_back(Scale(datalog_msg_.bfs_ins_east_vel_mps, -60.0f, 60.0f, 136.525f, 8191.5f));
        bfs_ins_down_vel_mps.push_back(Scale(datalog_msg_.bfs_ins_down_vel_mps, -60.0f, 60.0f, 136.525f, 8191.5f));
        bfs_ins_lat_rad.push_back(Scale(datalog_msg_.bfs_ins_lat_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5));
        bfs_ins_lon_rad.push_back(Scale(datalog_msg_.bfs_ins_lon_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5));
        #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
            defined(__FMU_R_V2_BETA__)
        vector_nav_ins_initialized.push_back(datalog_msg_.vector_nav_ins_initialized);
        vector_nav_ins_pitch_rad.push_back(Scale(datalog_msg_.vector_nav_ins_pitch_rad, -bfs::BFS_PI<float> / 2.0f, bfs::BFS_PI<float> / 2.0f, 20860.4383910547f, 32767.5f));
        vector_nav_ins_roll_rad.push_back(Scale(datalog_msg_.vector_nav_ins_roll_rad, -bfs::BFS_PI<float> / 2.0f, bfs::BFS_PI<float> / 2.0f, 20860.4383910547f, 32767.5f));
        vector_nav_ins_heading_rad.push_back(Scale(datalog_msg_.vector_nav_ins_heading_rad, -bfs::BFS_PI<float>, bfs::BFS_PI<float>, 10430.2191955274f, 32767.5f));
        vector_nav_ins_alt_wgs84_m.push_back(Scale(datalog_msg_.vector_nav_ins_alt_wgs84_m, -500.0f, 5000.0f, 190.65f, 95324.9999999999f));
        vector_nav_ins_accel_x_mps2.push_back(Scale(datalog_msg_.vector_nav_ins_accel_x_mps2, -156.9064f, 156.9064f, 208.834693804714f, 32767.5f));
        vector_nav_ins_accel_y_mps2.push_back(Scale(datalog_msg_.vector_nav_ins_accel_y_mps2, -156.9064f, 156.9064f, 208.834693804714f, 32767.5f));
        vector_nav_ins_accel_z_mps2.push_back(Scale(datalog_msg_.vector_nav_ins_accel_z_mps2, -156.9064f, 156.9064f, 208.834693804714f, 32767.5f));
        vector_nav_ins_gyro_x_radps.push_back(Scale(datalog_msg_.vector_nav_ins_gyro_x_radps, -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f));
        vector_nav_ins_gyro_y_radps.push_back(Scale(datalog_msg_.vector_nav_ins_gyro_y_radps, -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f));
        vector_nav_ins_gyro_z_radps.push_back(Scale(datalog_msg_.vector_nav_ins_gyro_z_radps, -34.9065850398866f, 34.9065850398866f, 938.719727597462f, 32767.5f));
        vector_nav_ins_mag_x_ut.push_back(Scale(datalog_msg_.vector_nav_ins_mag_x_ut, -250.0f, 250.0f, 131.07f, 32767.5f));
        vector_nav_ins_mag_y_ut.push_back(Scale(datalog_msg_.vector_nav_ins_mag_y_ut, -250.0f, 250.0f, 131.07f, 32767.5f));
        vector_nav_ins_mag_z_ut.push_back(Scale(datalog_msg_.vector_nav_ins_mag_z_ut, -250.0f, 250.0f, 131.07f, 32767.5f));
        vector_nav_ins_north_vel_mps.push_back(Scale(datalog_msg_.vector_nav_ins_north_vel_mps, -60.0f, 60.0f, 136.525f, 8191.5f));
        vector_nav_ins_east_vel_mps.push_back(Scale(datalog_msg_.vector_nav_ins_east_vel_mps, -60.0f, 60.0f, 136.525f, 8191.5f));
        vector_nav_ins_down_vel_mps.push_back(Scale(datalog_msg_.vector_nav_ins_down_vel_mps, -60.0f, 60.0f, 136.525f, 8191.5f));
        vector_nav_ins_lat_rad.push_back(Scale(datalog_msg_.vector_nav_ins_lat_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5));
        vector_nav_ins_lon_rad.push_back(Scale(datalog_msg_.vector_nav_ins_lon_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5));
        #endif
        aux_ins_home_alt_wgs84_m.push_back(Scale(datalog_msg_.aux_ins_home_alt_wgs84_m, -500.0f, 5000.0f, 190.65f, 95324.9999999999f));
        aux_ins_gnd_spd_mps.push_back(Scale(datalog_msg_.aux_ins_gnd_spd_mps, 0.0f, 60.0f, 136.516666666667f, 0.0f));
        aux_ins_gnd_track_rad.push_back(Scale(datalog_msg_.aux_ins_gnd_track_rad,  -bfs::BFS_PI<float>, bfs::BFS_PI<float>, 10430.2191955274f, 32767.5f));
        aux_ins_flight_path_rad.push_back(Scale(datalog_msg_.aux_ins_flight_path_rad, -bfs::BFS_PI<float> / 2.0f, bfs::BFS_PI<float> / 2.0f, 10430.0600405843f, 16383.5f));
        aux_ins_home_lat_rad.push_back(Scale(datalog_msg_.aux_ins_home_lat_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5));
        aux_ins_home_lon_rad.push_back(Scale(datalog_msg_.aux_ins_home_lon_rad, -bfs::BFS_PI<double>, bfs::BFS_PI<double>, 10937044409.0637, 34359738367.5));
        aux_ins_ned_pos_north_m.push_back(Scale(datalog_msg_.aux_ins_ned_pos_north_m, -50000.0, 50000.0, 1342.17727, 67108863.5));
        aux_ins_ned_pos_east_m.push_back(Scale(datalog_msg_.aux_ins_ned_pos_east_m, -50000.0, 50000.0, 1342.17727, 67108863.5));
        aux_ins_ned_pos_down_m.push_back(Scale(datalog_msg_.aux_ins_ned_pos_down_m, -5000.0, 500.0, 1525.20127272727, 7626006.36363636));
        adc_static_pres_pa.push_back(Scale(datalog_msg_.adc_static_pres_pa, 60000.0f, 120000.0f, 1.09225f, -65535.0f));
        adc_diff_pres_pa.push_back(Scale(datalog_msg_.adc_diff_pres_pa, 0.0f, 2000.0f, 32.7675f, 0.0f));
        adc_pres_alt_m.push_back(Scale(datalog_msg_.adc_pres_alt_m, -500.0f, 5000.0f, 11.9154545454545f, 5957.72727272727f));
        adc_rel_alt_m.push_back(Scale(datalog_msg_.adc_rel_alt_m, -500.0f, 5000.0f, 11.9154545454545f, 5957.72727272727f));
        adc_ias_mps.push_back(Scale(datalog_msg_.adc_ias_mps, 0.0f, 60.0f, 17.05f, 0.0f));
        telem_waypoint_frame.push_back(datalog_msg_.telem_waypoint_frame);
        telem_waypoint_cmd.push_back(datalog_msg_.telem_waypoint_cmd);
        temp_ = datalog_msg_.telem_waypoint_param1;
        memcpy(&tempf_, &temp_, 4);
        telem_waypoint_param1.push_back(tempf_);
        temp_ = datalog_msg_.telem_waypoint_param2;
        memcpy(&tempf_, &temp_, 4);
        telem_waypoint_param2.push_back(tempf_);
        temp_ = datalog_msg_.telem_waypoint_param3;
        memcpy(&tempf_, &temp_, 4);
        telem_waypoint_param3.push_back(tempf_);
        temp_ = datalog_msg_.telem_waypoint_param4;
        memcpy(&tempf_, &temp_, 4);
        telem_waypoint_param4.push_back(tempf_);
        temp_ = datalog_msg_.telem_waypoint_x;
        memcpy(&tempi_, &temp_, 4);
        telem_waypoint_x.push_back(tempi_);
        temp_ = datalog_msg_.telem_waypoint_y;
        memcpy(&tempi_, &temp_, 4);
        telem_waypoint_y.push_back(tempi_);
        temp_ = datalog_msg_.telem_waypoint_z;
        memcpy(&tempf_, &temp_, 4);
        telem_waypoint_z.push_back(tempf_);
        temp_ = datalog_msg_.telem_param0;
        memcpy(&tempf_, &temp_, 4);
        telem_param0.push_back(tempf_);
        temp_ = datalog_msg_.telem_param1;
        memcpy(&tempf_, &temp_, 4);
        telem_param1.push_back(tempf_);
        temp_ = datalog_msg_.telem_param2;
        memcpy(&tempf_, &temp_, 4);
        telem_param2.push_back(tempf_);
        temp_ = datalog_msg_.telem_param3;
        memcpy(&tempf_, &temp_, 4);
        telem_param3.push_back(tempf_);
        temp_ = datalog_msg_.telem_param4;
        memcpy(&tempf_, &temp_, 4);
        telem_param4.push_back(tempf_);
        temp_ = datalog_msg_.telem_param5;
        memcpy(&tempf_, &temp_, 4);
        telem_param5.push_back(tempf_);
        temp_ = datalog_msg_.telem_param6;
        memcpy(&tempf_, &temp_, 4);
        telem_param6.push_back(tempf_);
        temp_ = datalog_msg_.telem_param7;
        memcpy(&tempf_, &temp_, 4);
        telem_param7.push_back(tempf_);
        temp_ = datalog_msg_.telem_param8;
        memcpy(&tempf_, &temp_, 4);
        telem_param8.push_back(tempf_);
        temp_ = datalog_msg_.telem_param9;
        memcpy(&tempf_, &temp_, 4);
        telem_param9.push_back(tempf_);
        temp_ = datalog_msg_.telem_param10;
        memcpy(&tempf_, &temp_, 4);
        telem_param10.push_back(tempf_);
        temp_ = datalog_msg_.telem_param11;
        memcpy(&tempf_, &temp_, 4);
        telem_param11.push_back(tempf_);
        temp_ = datalog_msg_.telem_param12;
        memcpy(&tempf_, &temp_, 4);
        telem_param12.push_back(tempf_);
        temp_ = datalog_msg_.telem_param13;
        memcpy(&tempf_, &temp_, 4);
        telem_param13.push_back(tempf_);
        temp_ = datalog_msg_.telem_param14;
        memcpy(&tempf_, &temp_, 4);
        telem_param14.push_back(tempf_);
        temp_ = datalog_msg_.telem_param15;
        memcpy(&tempf_, &temp_, 4);
        telem_param15.push_back(tempf_);
        temp_ = datalog_msg_.telem_param16;
        memcpy(&tempf_, &temp_, 4);
        telem_param16.push_back(tempf_);
        temp_ = datalog_msg_.telem_param17;
        memcpy(&tempf_, &temp_, 4);
        telem_param17.push_back(tempf_);
        temp_ = datalog_msg_.telem_param18;
        memcpy(&tempf_, &temp_, 4);
        telem_param18.push_back(tempf_);
        temp_ = datalog_msg_.telem_param19;
        memcpy(&tempf_, &temp_, 4);
        telem_param19.push_back(tempf_);
        temp_ = datalog_msg_.telem_param20;
        memcpy(&tempf_, &temp_, 4);
        telem_param20.push_back(tempf_);
        temp_ = datalog_msg_.telem_param21;
        memcpy(&tempf_, &temp_, 4);
        telem_param21.push_back(tempf_);
        temp_ = datalog_msg_.telem_param22;
        memcpy(&tempf_, &temp_, 4);
        telem_param22.push_back(tempf_);
        temp_ = datalog_msg_.telem_param23;
        memcpy(&tempf_, &temp_, 4);
        telem_param23.push_back(tempf_);
        vms_advance_waypoint.push_back(datalog_msg_.vms_advance_waypoint);
        vms_motors_enabled.push_back(datalog_msg_.vms_motors_enabled);
        vms_mode.push_back(datalog_msg_.vms_mode);
        vms_sbus_cmd0.push_back(datalog_msg_.vms_sbus_cmd0);
        vms_sbus_cmd1.push_back(datalog_msg_.vms_sbus_cmd1);
        vms_sbus_cmd2.push_back(datalog_msg_.vms_sbus_cmd2);
        vms_sbus_cmd3.push_back(datalog_msg_.vms_sbus_cmd3);
        vms_sbus_cmd4.push_back(datalog_msg_.vms_sbus_cmd4);
        vms_sbus_cmd5.push_back(datalog_msg_.vms_sbus_cmd5);
        vms_sbus_cmd6.push_back(datalog_msg_.vms_sbus_cmd6);
        vms_sbus_cmd7.push_back(datalog_msg_.vms_sbus_cmd7);
        vms_sbus_cmd8.push_back(datalog_msg_.vms_sbus_cmd8);
        vms_sbus_cmd9.push_back(datalog_msg_.vms_sbus_cmd9);
        vms_sbus_cmd10.push_back(datalog_msg_.vms_sbus_cmd10);
        vms_sbus_cmd11.push_back(datalog_msg_.vms_sbus_cmd11);
        vms_sbus_cmd12.push_back(datalog_msg_.vms_sbus_cmd12);
        vms_sbus_cmd13.push_back(datalog_msg_.vms_sbus_cmd13);
        vms_sbus_cmd14.push_back(datalog_msg_.vms_sbus_cmd14);
        vms_sbus_cmd15.push_back(datalog_msg_.vms_sbus_cmd15);
        vms_pwm_cmd0.push_back(datalog_msg_.vms_pwm_cmd0 + 1000);
        vms_pwm_cmd1.push_back(datalog_msg_.vms_pwm_cmd1 + 1000);
        vms_pwm_cmd2.push_back(datalog_msg_.vms_pwm_cmd2 + 1000);
        vms_pwm_cmd3.push_back(datalog_msg_.vms_pwm_cmd3 + 1000);
        vms_pwm_cmd4.push_back(datalog_msg_.vms_pwm_cmd4 + 1000);
        vms_pwm_cmd5.push_back(datalog_msg_.vms_pwm_cmd5 + 1000);
        vms_pwm_cmd6.push_back(datalog_msg_.vms_pwm_cmd6 + 1000);
        vms_pwm_cmd7.push_back(datalog_msg_.vms_pwm_cmd7 + 1000);
        vms_flight_time_remaining_s.push_back(Scale(datalog_msg_.vms_flight_time_remaining_s, 0.0f, 32400.0f, 2.02268518518519f, 0.0f));
        vms_power_remaining_prcnt.push_back(Scale(datalog_msg_.vms_power_remaining_prcnt, 0.0f, 100.0f, 10.23f, 0.0f));
        vms_throttle_cmd_prcnt.push_back(Scale(datalog_msg_.vms_throttle_prcnt, 0.0f, 100.0f, 10.23f, 0.0f));
        temp_ = datalog_msg_.vms_aux0;
        memcpy(&tempf_, &temp_, 4);
        vms_aux0.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux1;
        memcpy(&tempf_, &temp_, 4);
        vms_aux1.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux2;
        memcpy(&tempf_, &temp_, 4);
        vms_aux2.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux3;
        memcpy(&tempf_, &temp_, 4);
        vms_aux3.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux4;
        memcpy(&tempf_, &temp_, 4);
        vms_aux4.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux5;
        memcpy(&tempf_, &temp_, 4);
        vms_aux5.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux6;
        memcpy(&tempf_, &temp_, 4);
        vms_aux6.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux7;
        memcpy(&tempf_, &temp_, 4);
        vms_aux7.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux8;
        memcpy(&tempf_, &temp_, 4);
        vms_aux8.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux9;
        memcpy(&tempf_, &temp_, 4);
        vms_aux9.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux10;
        memcpy(&tempf_, &temp_, 4);
        vms_aux10.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux11;
        memcpy(&tempf_, &temp_, 4);
        vms_aux11.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux12;
        memcpy(&tempf_, &temp_, 4);
        vms_aux12.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux13;
        memcpy(&tempf_, &temp_, 4);
        vms_aux13.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux14;
        memcpy(&tempf_, &temp_, 4);
        vms_aux14.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux15;
        memcpy(&tempf_, &temp_, 4);
        vms_aux15.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux16;
        memcpy(&tempf_, &temp_, 4);
        vms_aux16.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux17;
        memcpy(&tempf_, &temp_, 4);
        vms_aux17.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux18;
        memcpy(&tempf_, &temp_, 4);
        vms_aux18.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux19;
        memcpy(&tempf_, &temp_, 4);
        vms_aux19.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux20;
        memcpy(&tempf_, &temp_, 4);
        vms_aux20.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux21;
        memcpy(&tempf_, &temp_, 4);
        vms_aux21.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux22;
        memcpy(&tempf_, &temp_, 4);
        vms_aux22.push_back(tempf_);
        temp_ = datalog_msg_.vms_aux23;
        memcpy(&tempf_, &temp_, 4);
        vms_aux23.push_back(tempf_);
      }
    }
  }
  bfs::MatWrite("sys_frame_time_s", sys_frame_time_s, output);
  #if defined(__FMU_R_V1__)
  bfs::MatWrite("sys_input_volt", sys_input_volt, output);
  bfs::MatWrite("sys_reg_volt", sys_reg_volt, output);
  bfs::MatWrite("sys_pwm_volt", sys_pwm_volt, output);
  bfs::MatWrite("sys_sbus_volt", sys_sbus_volt, output);
  #endif
  bfs::MatWrite("sys_time_s", sys_time_s, output);
  bfs::MatWrite("incept_new_data", incept_new_data, output);
  bfs::MatWrite("incept_lost_frame", incept_lost_frame, output);
  bfs::MatWrite("incept_failsafe", incept_failsafe, output);
  bfs::MatWrite("incept_ch0", incept_ch0, output);
  bfs::MatWrite("incept_ch1", incept_ch1, output);
  bfs::MatWrite("incept_ch2", incept_ch2, output);
  bfs::MatWrite("incept_ch3", incept_ch3, output);
  bfs::MatWrite("incept_ch4", incept_ch4, output);
  bfs::MatWrite("incept_ch5", incept_ch5, output);
  bfs::MatWrite("incept_ch6", incept_ch6, output);
  bfs::MatWrite("incept_ch7", incept_ch7, output);
  bfs::MatWrite("incept_ch8", incept_ch8, output);
  bfs::MatWrite("incept_ch9", incept_ch9, output);
  bfs::MatWrite("incept_ch10", incept_ch10, output);
  bfs::MatWrite("incept_ch11", incept_ch11, output);
  bfs::MatWrite("incept_ch12", incept_ch12, output);
  bfs::MatWrite("incept_ch13", incept_ch13, output);
  bfs::MatWrite("incept_ch14", incept_ch14, output);
  bfs::MatWrite("incept_ch15", incept_ch15, output);
  bfs::MatWrite("fmu_imu_healthy", fmu_imu_healthy, output);
  bfs::MatWrite("fmu_imu_new_data", fmu_imu_new_data, output);
  bfs::MatWrite("fmu_imu_die_temp_c", fmu_imu_die_temp_c, output);
  bfs::MatWrite("fmu_imu_accel_x_mps2", fmu_imu_accel_x_mps2, output);
  bfs::MatWrite("fmu_imu_accel_y_mps2", fmu_imu_accel_y_mps2, output);
  bfs::MatWrite("fmu_imu_accel_z_mps2", fmu_imu_accel_z_mps2, output);
  bfs::MatWrite("fmu_imu_gyro_x_radps", fmu_imu_gyro_x_radps, output);
  bfs::MatWrite("fmu_imu_gyro_y_radps", fmu_imu_gyro_y_radps, output);
  bfs::MatWrite("fmu_imu_gyro_z_radps", fmu_imu_gyro_z_radps, output);
  bfs::MatWrite("fmu_mag_healthy", fmu_mag_healthy, output);
  bfs::MatWrite("fmu_mag_new_data", fmu_mag_new_data, output);
  bfs::MatWrite("fmu_mag_die_temp_c", fmu_mag_die_temp_c, output);
  bfs::MatWrite("fmu_mag_x_ut", fmu_mag_x_ut, output);
  bfs::MatWrite("fmu_mag_y_ut", fmu_mag_y_ut, output);
  bfs::MatWrite("fmu_mag_z_ut", fmu_mag_z_ut, output);
  bfs::MatWrite("fmu_static_pres_healthy", fmu_static_pres_healthy, output);
  bfs::MatWrite("fmu_static_pres_new_data", fmu_static_pres_new_data, output);
  bfs::MatWrite("fmu_static_pres_die_temp_c", fmu_static_pres_die_temp_c, output);
  bfs::MatWrite("fmu_static_pres_pa", fmu_static_pres_pa, output);
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  bfs::MatWrite("vector_nav_installed", vector_nav_installed, output);
  bfs::MatWrite("vector_nav_healthy", vector_nav_healthy, output);
  bfs::MatWrite("vector_nav_die_temp_c", vector_nav_die_temp_c, output);
  bfs::MatWrite("vector_nav_imu_new_data", vector_nav_imu_new_data, output);
  bfs::MatWrite("vector_nav_imu_accel_x_mps2", vector_nav_imu_accel_x_mps2, output);
  bfs::MatWrite("vector_nav_imu_accel_y_mps2", vector_nav_imu_accel_y_mps2, output);
  bfs::MatWrite("vector_nav_imu_accel_z_mps2", vector_nav_imu_accel_z_mps2, output);
  bfs::MatWrite("vector_nav_imu_gyro_x_radps", vector_nav_imu_gyro_x_radps, output);
  bfs::MatWrite("vector_nav_imu_gyro_y_radps", vector_nav_imu_gyro_y_radps, output);
  bfs::MatWrite("vector_nav_imu_gyro_z_radps", vector_nav_imu_gyro_z_radps, output);
  bfs::MatWrite("vector_nav_mag_x_ut", vector_nav_mag_x_ut, output);
  bfs::MatWrite("vector_nav_mag_y_ut", vector_nav_mag_y_ut, output);
  bfs::MatWrite("vector_nav_mag_z_ut", vector_nav_mag_z_ut, output);
  bfs::MatWrite("vector_nav_static_pres_pa", vector_nav_static_pres_pa, output);
  bfs::MatWrite("vector_nav_gnss_installed", vector_nav_gnss_installed, output);
  bfs::MatWrite("vector_nav_gnss_healthy", vector_nav_gnss_healthy, output);
  bfs::MatWrite("vector_nav_gnss_new_data", vector_nav_gnss_new_data, output);
  bfs::MatWrite("vector_nav_gnss_fix", vector_nav_gnss_fix, output);
  bfs::MatWrite("vector_nav_gnss_num_sats", vector_nav_gnss_num_sats, output);
  bfs::MatWrite("vector_nav_gnss_gps_week", vector_nav_gnss_gps_week, output);
  bfs::MatWrite("vector_nav_gnss_alt_wgs84", vector_nav_gnss_alt_wgs84, output);
  bfs::MatWrite("vector_nav_gnss_horz_acc_m", vector_nav_gnss_horz_acc_m, output);
  bfs::MatWrite("vector_nav_gnss_vert_acc_m", vector_nav_gnss_vert_acc_m, output);
  bfs::MatWrite("vector_nav_gnss_vel_acc_mps", vector_nav_gnss_vel_acc_mps, output);
  bfs::MatWrite("vector_nav_gnss_north_vel_mps", vector_nav_gnss_north_vel_mps, output);
  bfs::MatWrite("vector_nav_gnss_east_vel_mps", vector_nav_gnss_east_vel_mps, output);
  bfs::MatWrite("vector_nav_gnss_down_vel_mps", vector_nav_gnss_down_vel_mps, output);
  bfs::MatWrite("vector_nav_gnss_lat_rad", vector_nav_gnss_lat_rad, output);
  bfs::MatWrite("vector_nav_gnss_lon_rad", vector_nav_gnss_lon_rad, output);
  bfs::MatWrite("vector_nav_gnss_gps_tow_s", vector_nav_gnss_gps_tow_s, output);
  #endif
  bfs::MatWrite("ext_mag_installed", ext_mag_installed, output);
  bfs::MatWrite("ext_mag_healthy", ext_mag_healthy, output);
  bfs::MatWrite("ext_mag_new_data", ext_mag_new_data, output);
  bfs::MatWrite("ext_mag_die_temp_c", ext_mag_die_temp_c, output);
  bfs::MatWrite("ext_mag_x_ut", ext_mag_x_ut, output);
  bfs::MatWrite("ext_mag_y_ut", ext_mag_y_ut, output);
  bfs::MatWrite("ext_mag_z_ut", ext_mag_z_ut, output);
  bfs::MatWrite("ext_gnss1_installed", ext_gnss1_installed, output);
  bfs::MatWrite("ext_gnss1_healthy", ext_gnss1_healthy, output);
  bfs::MatWrite("ext_gnss1_new_data", ext_gnss1_new_data, output);
  bfs::MatWrite("ext_gnss1_rel_pos_avail", ext_gnss1_rel_pos_avail, output);
  bfs::MatWrite("ext_gnss1_rel_pos_moving_baseline", ext_gnss1_rel_pos_moving_baseline, output);
  bfs::MatWrite("ext_gnss1_rel_pos_baseline_normalized", ext_gnss1_rel_pos_baseline_normalized, output);
  bfs::MatWrite("ext_gnss1_fix", ext_gnss1_fix, output);
  bfs::MatWrite("ext_gnss1_num_sats", ext_gnss1_num_sats, output);
  bfs::MatWrite("ext_gnss1_gps_week", ext_gnss1_gps_week, output);
  bfs::MatWrite("ext_gnss1_alt_wgs84_m", ext_gnss1_alt_wgs84_m, output);
  bfs::MatWrite("ext_gnss1_horz_acc_m", ext_gnss1_horz_acc_m, output);
  bfs::MatWrite("ext_gnss1_vert_acc_m", ext_gnss1_vert_acc_m, output);
  bfs::MatWrite("ext_gnss1_vel_acc_mps", ext_gnss1_vel_acc_mps, output);
  bfs::MatWrite("ext_gnss1_north_vel_mps", ext_gnss1_north_vel_mps, output);
  bfs::MatWrite("ext_gnss1_east_vel_mps", ext_gnss1_east_vel_mps, output);
  bfs::MatWrite("ext_gnss1_down_vel_mps", ext_gnss1_down_vel_mps, output);
  bfs::MatWrite("ext_gnss1_rel_pos_acc_north_m", ext_gnss1_rel_pos_acc_north_m, output);
  bfs::MatWrite("ext_gnss1_rel_pos_acc_east_m", ext_gnss1_rel_pos_acc_east_m, output);
  bfs::MatWrite("ext_gnss1_rel_pos_acc_down_m", ext_gnss1_rel_pos_acc_down_m, output);
  bfs::MatWrite("ext_gnss1_gps_tow_s", ext_gnss1_gps_tow_s, output);
  bfs::MatWrite("ext_gnss1_lat_rad", ext_gnss1_lat_rad, output);
  bfs::MatWrite("ext_gnss1_lon_rad", ext_gnss1_lon_rad, output);
  bfs::MatWrite("ext_gnss1_rel_pos_north_m", ext_gnss1_rel_pos_north_m, output);
  bfs::MatWrite("ext_gnss1_rel_pos_east_m", ext_gnss1_rel_pos_east_m, output);
  bfs::MatWrite("ext_gnss1_rel_pos_down_m", ext_gnss1_rel_pos_down_m, output);
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  bfs::MatWrite("ext_gnss2_installed", ext_gnss2_installed, output);
  bfs::MatWrite("ext_gnss2_healthy", ext_gnss2_healthy, output);
  bfs::MatWrite("ext_gnss2_new_data", ext_gnss2_new_data, output);
  bfs::MatWrite("ext_gnss2_rel_pos_avail", ext_gnss2_rel_pos_avail, output);
  bfs::MatWrite("ext_gnss2_rel_pos_moving_baseline", ext_gnss2_rel_pos_moving_baseline, output);
  bfs::MatWrite("ext_gnss2_rel_pos_baseline_normalized", ext_gnss2_rel_pos_baseline_normalized, output);
  bfs::MatWrite("ext_gnss2_fix", ext_gnss2_fix, output);
  bfs::MatWrite("ext_gnss2_num_sats", ext_gnss2_num_sats, output);
  bfs::MatWrite("ext_gnss2_gps_week", ext_gnss2_gps_week, output);
  bfs::MatWrite("ext_gnss2_alt_wgs84_m", ext_gnss2_alt_wgs84_m, output);
  bfs::MatWrite("ext_gnss2_horz_acc_m", ext_gnss2_horz_acc_m, output);
  bfs::MatWrite("ext_gnss2_vert_acc_m", ext_gnss2_vert_acc_m, output);
  bfs::MatWrite("ext_gnss2_vel_acc_mps", ext_gnss2_vel_acc_mps, output);
  bfs::MatWrite("ext_gnss2_north_vel_mps", ext_gnss2_north_vel_mps, output);
  bfs::MatWrite("ext_gnss2_east_vel_mps", ext_gnss2_east_vel_mps, output);
  bfs::MatWrite("ext_gnss2_down_vel_mps", ext_gnss2_down_vel_mps, output);
  bfs::MatWrite("ext_gnss2_rel_pos_acc_north_m", ext_gnss2_rel_pos_acc_north_m, output);
  bfs::MatWrite("ext_gnss2_rel_pos_acc_east_m", ext_gnss2_rel_pos_acc_east_m, output);
  bfs::MatWrite("ext_gnss2_rel_pos_acc_down_m", ext_gnss2_rel_pos_acc_down_m, output);
  bfs::MatWrite("ext_gnss2_gps_tow_s", ext_gnss2_gps_tow_s, output);
  bfs::MatWrite("ext_gnss2_lat_rad", ext_gnss2_lat_rad, output);
  bfs::MatWrite("ext_gnss2_lon_rad", ext_gnss2_lon_rad, output);
  bfs::MatWrite("ext_gnss2_rel_pos_north_m", ext_gnss2_rel_pos_north_m, output);
  bfs::MatWrite("ext_gnss2_rel_pos_east_m", ext_gnss2_rel_pos_east_m, output);
  bfs::MatWrite("ext_gnss2_rel_pos_down_m", ext_gnss2_rel_pos_down_m, output);
  #endif
  bfs::MatWrite("ext_pres1_installed", ext_pres1_installed, output);
  bfs::MatWrite("ext_pres1_is_static_pres", ext_pres1_is_static_pres, output);
  bfs::MatWrite("ext_pres1_healthy", ext_pres1_healthy, output);
  bfs::MatWrite("ext_pres1_new_data", ext_pres1_new_data, output);
  bfs::MatWrite("ext_pres1_die_temp_c", ext_pres1_die_temp_c, output);
  bfs::MatWrite("ext_pres1_pres_pa", ext_pres1_pres_pa, output);
  bfs::MatWrite("ext_pres2_installed", ext_pres2_installed, output);
  bfs::MatWrite("ext_pres2_is_static_pres", ext_pres2_is_static_pres, output);
  bfs::MatWrite("ext_pres2_healthy", ext_pres2_healthy, output);
  bfs::MatWrite("ext_pres2_new_data", ext_pres2_new_data, output);
  bfs::MatWrite("ext_pres2_die_temp_c", ext_pres2_die_temp_c, output);
  bfs::MatWrite("ext_pres2_pres_pa", ext_pres2_pres_pa, output);
  bfs::MatWrite("ext_pres3_installed", ext_pres3_installed, output);
  bfs::MatWrite("ext_pres3_is_static_pres", ext_pres3_is_static_pres, output);
  bfs::MatWrite("ext_pres3_healthy", ext_pres3_healthy, output);
  bfs::MatWrite("ext_pres3_new_data", ext_pres3_new_data, output);
  bfs::MatWrite("ext_pres3_die_temp_c", ext_pres3_die_temp_c, output);
  bfs::MatWrite("ext_pres3_pres_pa", ext_pres3_pres_pa, output);
  bfs::MatWrite("ext_pres4_installed", ext_pres4_installed, output);
  bfs::MatWrite("ext_pres4_is_static_pres", ext_pres4_is_static_pres, output);
  bfs::MatWrite("ext_pres4_healthy", ext_pres4_healthy, output);
  bfs::MatWrite("ext_pres4_new_data", ext_pres4_new_data, output);
  bfs::MatWrite("ext_pres4_die_temp_c", ext_pres4_die_temp_c, output);
  bfs::MatWrite("ext_pres4_pres_pa", ext_pres4_pres_pa, output);
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  bfs::MatWrite("rad_alt_installed", rad_alt_installed, output);
  bfs::MatWrite("rad_alt_healthy", rad_alt_healthy, output);
  bfs::MatWrite("rad_alt_new_data", rad_alt_new_data, output);
  bfs::MatWrite("rad_alt_snr", rad_alt_snr, output);
  bfs::MatWrite("rad_alt_alt_m", rad_alt_alt_m, output);

  bfs::MatWrite("opflow_installed", opflow_installed, output);
  bfs::MatWrite("opflow_healthy", opflow_healthy, output);
  bfs::MatWrite("opflow_new_data", opflow_new_data, output);
  bfs::MatWrite("opflow_sur_qual", opflow_sur_qual, output);
  bfs::MatWrite("opflow_range_qual", opflow_range_qual, output);
  bfs::MatWrite("opflow_mot_x", opflow_mot_x, output);
  bfs::MatWrite("opflow_mot_y", opflow_mot_y, output);
  bfs::MatWrite("opflow_range_mm", opflow_range_mm, output);
  #endif
  bfs::MatWrite("ain0_v", ain0_v, output);
  bfs::MatWrite("ain1_v", ain1_v, output);
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  bfs::MatWrite("ain2_v", ain2_v, output);
  bfs::MatWrite("ain3_v", ain3_v, output);
  bfs::MatWrite("ain4_v", ain4_v, output);
  bfs::MatWrite("ain5_v", ain5_v, output);
  #endif
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__)
  bfs::MatWrite("ain6_v", ain6_v, output);
  bfs::MatWrite("ain7_v", ain7_v, output);
  #endif
  #if defined(__FMU_R_V2__) || defined(__FMU_R_MINI_V1__)
  bfs::MatWrite("power_module_voltage_v", power_module_voltage_v, output);
  bfs::MatWrite("power_module_current_ma", power_module_current_ma, output);
  #endif
  bfs::MatWrite("bfs_ins_initialized", bfs_ins_initialized, output);
  bfs::MatWrite("bfs_ins_pitch_rad", bfs_ins_pitch_rad, output);
  bfs::MatWrite("bfs_ins_roll_rad", bfs_ins_roll_rad, output);
  bfs::MatWrite("bfs_ins_heading_rad", bfs_ins_heading_rad, output);
  bfs::MatWrite("bfs_ins_alt_wgs84_m", bfs_ins_alt_wgs84_m, output);
  bfs::MatWrite("bfs_ins_accel_x_mps2", bfs_ins_accel_x_mps2, output);
  bfs::MatWrite("bfs_ins_accel_y_mps2", bfs_ins_accel_y_mps2, output);
  bfs::MatWrite("bfs_ins_accel_z_mps2", bfs_ins_accel_z_mps2, output);
  bfs::MatWrite("bfs_ins_gyro_x_radps", bfs_ins_gyro_x_radps, output);
  bfs::MatWrite("bfs_ins_gyro_y_radps", bfs_ins_gyro_y_radps, output);
  bfs::MatWrite("bfs_ins_gyro_z_radps", bfs_ins_gyro_z_radps, output);
  bfs::MatWrite("bfs_ins_mag_x_ut", bfs_ins_mag_x_ut, output);
  bfs::MatWrite("bfs_ins_mag_y_ut", bfs_ins_mag_y_ut, output);
  bfs::MatWrite("bfs_ins_mag_z_ut", bfs_ins_mag_z_ut, output);
  bfs::MatWrite("bfs_ins_north_vel_mps", bfs_ins_north_vel_mps, output);
  bfs::MatWrite("bfs_ins_east_vel_mps", bfs_ins_east_vel_mps, output);
  bfs::MatWrite("bfs_ins_down_vel_mps", bfs_ins_down_vel_mps, output);
  bfs::MatWrite("bfs_ins_lat_rad", bfs_ins_lat_rad, output);
  bfs::MatWrite("bfs_ins_lon_rad", bfs_ins_lon_rad, output);
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  bfs::MatWrite("vector_nav_ins_initialized", vector_nav_ins_initialized, output);
  bfs::MatWrite("vector_nav_ins_pitch_rad", vector_nav_ins_pitch_rad, output);
  bfs::MatWrite("vector_nav_ins_roll_rad", vector_nav_ins_roll_rad, output);
  bfs::MatWrite("vector_nav_ins_heading_rad", vector_nav_ins_heading_rad, output);
  bfs::MatWrite("vector_nav_ins_alt_wgs84_m", vector_nav_ins_alt_wgs84_m, output);
  bfs::MatWrite("vector_nav_ins_accel_x_mps2", vector_nav_ins_accel_x_mps2, output);
  bfs::MatWrite("vector_nav_ins_accel_y_mps2", vector_nav_ins_accel_y_mps2, output);
  bfs::MatWrite("vector_nav_ins_accel_z_mps2", vector_nav_ins_accel_z_mps2, output);
  bfs::MatWrite("vector_nav_ins_gyro_x_radps", vector_nav_ins_gyro_x_radps, output);
  bfs::MatWrite("vector_nav_ins_gyro_y_radps", vector_nav_ins_gyro_y_radps, output);
  bfs::MatWrite("vector_nav_ins_gyro_z_radps", vector_nav_ins_gyro_z_radps, output);
  bfs::MatWrite("vector_nav_ins_mag_x_ut", vector_nav_ins_mag_x_ut, output);
  bfs::MatWrite("vector_nav_ins_mag_y_ut", vector_nav_ins_mag_y_ut, output);
  bfs::MatWrite("vector_nav_ins_mag_z_ut", vector_nav_ins_mag_z_ut, output);
  bfs::MatWrite("vector_nav_ins_north_vel_mps", vector_nav_ins_north_vel_mps, output);
  bfs::MatWrite("vector_nav_ins_east_vel_mps", vector_nav_ins_east_vel_mps, output);
  bfs::MatWrite("vector_nav_ins_down_vel_mps", vector_nav_ins_down_vel_mps, output);
  bfs::MatWrite("vector_nav_ins_lat_rad", vector_nav_ins_lat_rad, output);
  bfs::MatWrite("vector_nav_ins_lon_rad", vector_nav_ins_lon_rad, output);
  #endif
  bfs::MatWrite("aux_ins_home_alt_wgs84_m", aux_ins_home_alt_wgs84_m, output);
  bfs::MatWrite("aux_ins_gnd_spd_mps", aux_ins_gnd_spd_mps, output);
  bfs::MatWrite("aux_ins_gnd_track_rad", aux_ins_gnd_track_rad, output);
  bfs::MatWrite("aux_ins_flight_path_rad", aux_ins_flight_path_rad, output);
  bfs::MatWrite("aux_ins_home_lat_rad", aux_ins_home_lat_rad, output);
  bfs::MatWrite("aux_ins_home_lon_rad", aux_ins_home_lon_rad, output);
  bfs::MatWrite("aux_ins_ned_pos_north_m", aux_ins_ned_pos_north_m, output);
  bfs::MatWrite("aux_ins_ned_pos_east_m", aux_ins_ned_pos_east_m, output);
  bfs::MatWrite("aux_ins_ned_pos_down_m", aux_ins_ned_pos_down_m, output);
  bfs::MatWrite("adc_static_pres_pa", adc_static_pres_pa, output);
  bfs::MatWrite("adc_diff_pres_pa", adc_diff_pres_pa, output);
  bfs::MatWrite("adc_pres_alt_m", adc_pres_alt_m, output);
  bfs::MatWrite("adc_rel_alt_m", adc_rel_alt_m, output);
  bfs::MatWrite("adc_ias_mps", adc_ias_mps, output);
  bfs::MatWrite("telem_waypoint_frame", telem_waypoint_frame, output);
  bfs::MatWrite("telem_waypoint_cmd", telem_waypoint_cmd, output);
  bfs::MatWrite("telem_waypoint_param1", telem_waypoint_param1, output);
  bfs::MatWrite("telem_waypoint_param2", telem_waypoint_param2, output);
  bfs::MatWrite("telem_waypoint_param3", telem_waypoint_param3, output);
  bfs::MatWrite("telem_waypoint_param4", telem_waypoint_param4, output);
  bfs::MatWrite("telem_waypoint_x", telem_waypoint_x, output);
  bfs::MatWrite("telem_waypoint_y", telem_waypoint_y, output);
  bfs::MatWrite("telem_waypoint_z", telem_waypoint_z, output);
  bfs::MatWrite("telem_param0", telem_param0, output);
  bfs::MatWrite("telem_param1", telem_param1, output);
  bfs::MatWrite("telem_param2", telem_param2, output);
  bfs::MatWrite("telem_param3", telem_param3, output);
  bfs::MatWrite("telem_param4", telem_param4, output);
  bfs::MatWrite("telem_param5", telem_param5, output);
  bfs::MatWrite("telem_param6", telem_param6, output);
  bfs::MatWrite("telem_param7", telem_param7, output);
  bfs::MatWrite("telem_param8", telem_param8, output);
  bfs::MatWrite("telem_param9", telem_param9, output);
  bfs::MatWrite("telem_param10", telem_param10, output);
  bfs::MatWrite("telem_param11", telem_param11, output);
  bfs::MatWrite("telem_param12", telem_param12, output);
  bfs::MatWrite("telem_param13", telem_param13, output);
  bfs::MatWrite("telem_param14", telem_param14, output);
  bfs::MatWrite("telem_param15", telem_param15, output);
  bfs::MatWrite("telem_param16", telem_param16, output);
  bfs::MatWrite("telem_param17", telem_param17, output);
  bfs::MatWrite("telem_param18", telem_param18, output);
  bfs::MatWrite("telem_param19", telem_param19, output);
  bfs::MatWrite("telem_param20", telem_param20, output);
  bfs::MatWrite("telem_param21", telem_param21, output);
  bfs::MatWrite("telem_param22", telem_param22, output);
  bfs::MatWrite("telem_param23", telem_param23, output);
  bfs::MatWrite("vms_advance_waypoint", vms_advance_waypoint, output);
  bfs::MatWrite("vms_motors_enabled", vms_motors_enabled, output);
  bfs::MatWrite("vms_mode", vms_mode, output);
  bfs::MatWrite("vms_sbus_cmd0", vms_sbus_cmd0, output);
  bfs::MatWrite("vms_sbus_cmd1", vms_sbus_cmd1, output);
  bfs::MatWrite("vms_sbus_cmd2", vms_sbus_cmd2, output);
  bfs::MatWrite("vms_sbus_cmd3", vms_sbus_cmd3, output);
  bfs::MatWrite("vms_sbus_cmd4", vms_sbus_cmd4, output);
  bfs::MatWrite("vms_sbus_cmd5", vms_sbus_cmd5, output);
  bfs::MatWrite("vms_sbus_cmd6", vms_sbus_cmd6, output);
  bfs::MatWrite("vms_sbus_cmd7", vms_sbus_cmd7, output);
  bfs::MatWrite("vms_sbus_cmd8", vms_sbus_cmd8, output);
  bfs::MatWrite("vms_sbus_cmd9", vms_sbus_cmd9, output);
  bfs::MatWrite("vms_sbus_cmd10", vms_sbus_cmd10, output);
  bfs::MatWrite("vms_sbus_cmd11", vms_sbus_cmd11, output);
  bfs::MatWrite("vms_sbus_cmd12", vms_sbus_cmd12, output);
  bfs::MatWrite("vms_sbus_cmd13", vms_sbus_cmd13, output);
  bfs::MatWrite("vms_sbus_cmd14", vms_sbus_cmd14, output);
  bfs::MatWrite("vms_sbus_cmd15", vms_sbus_cmd15, output);
  bfs::MatWrite("vms_pwm_cmd0", vms_pwm_cmd0, output);
  bfs::MatWrite("vms_pwm_cmd1", vms_pwm_cmd1, output);
  bfs::MatWrite("vms_pwm_cmd2", vms_pwm_cmd2, output);
  bfs::MatWrite("vms_pwm_cmd3", vms_pwm_cmd3, output);
  bfs::MatWrite("vms_pwm_cmd4", vms_pwm_cmd4, output);
  bfs::MatWrite("vms_pwm_cmd5", vms_pwm_cmd5, output);
  bfs::MatWrite("vms_pwm_cmd6", vms_pwm_cmd6, output);
  bfs::MatWrite("vms_pwm_cmd7", vms_pwm_cmd7, output);
  bfs::MatWrite("vms_flight_time_remaining_s", vms_flight_time_remaining_s, output);
  bfs::MatWrite("vms_power_remaining_prcnt", vms_power_remaining_prcnt, output);
  bfs::MatWrite("vms_throttle_cmd_prcnt", vms_throttle_cmd_prcnt, output);
  bfs::MatWrite("vms_aux0", vms_aux0, output);
  bfs::MatWrite("vms_aux1", vms_aux1, output);
  bfs::MatWrite("vms_aux2", vms_aux2, output);
  bfs::MatWrite("vms_aux3", vms_aux3, output);
  bfs::MatWrite("vms_aux4", vms_aux4, output);
  bfs::MatWrite("vms_aux5", vms_aux5, output);
  bfs::MatWrite("vms_aux6", vms_aux6, output);
  bfs::MatWrite("vms_aux7", vms_aux7, output);
  bfs::MatWrite("vms_aux8", vms_aux8, output);
  bfs::MatWrite("vms_aux9", vms_aux9, output);
  bfs::MatWrite("vms_aux10", vms_aux10, output);
  bfs::MatWrite("vms_aux11", vms_aux11, output);
  bfs::MatWrite("vms_aux12", vms_aux12, output);
  bfs::MatWrite("vms_aux13", vms_aux13, output);
  bfs::MatWrite("vms_aux14", vms_aux14, output);
  bfs::MatWrite("vms_aux15", vms_aux15, output);
  bfs::MatWrite("vms_aux16", vms_aux16, output);
  bfs::MatWrite("vms_aux17", vms_aux17, output);
  bfs::MatWrite("vms_aux18", vms_aux18, output);
  bfs::MatWrite("vms_aux19", vms_aux19, output);
  bfs::MatWrite("vms_aux20", vms_aux20, output);
  bfs::MatWrite("vms_aux21", vms_aux21, output);
  bfs::MatWrite("vms_aux22", vms_aux22, output);
  bfs::MatWrite("vms_aux23", vms_aux23, output);
  /* Print out closing info */
  std::cout << "done." << std::endl;
  std::cout << "Wrote " << sys_time_s.size() << " data packets." << std::endl;
  std::cout << "Saved as " << output_file_name << std::endl;
  return 0;
}

