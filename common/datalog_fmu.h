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

#ifndef COMMON_DATALOG_FMU_V1_H_
#define COMMON_DATALOG_FMU_V1_H_

#include <cinttypes>

struct DatalogMsg {
  uint64_t sys_frame_time_us : 15;
  #if defined(__FMU_R_V1__)
  uint64_t sys_input_volt : 12;
  uint64_t sys_reg_volt : 9;
  uint64_t sys_pwm_volt : 10;
  uint64_t sys_sbus_volt : 10;
  #endif
  uint64_t sys_time_us : 35;
  uint64_t incept_new_data : 1;
  uint64_t incept_lost_frame : 1;
  uint64_t incept_failsafe : 1; 
  uint64_t incept_ch0 : 11;
  uint64_t incept_ch1 : 11;
  uint64_t incept_ch2 : 11;
  uint64_t incept_ch3 : 11;
  uint64_t incept_ch4 : 11;
  uint64_t incept_ch5 : 11;
  uint64_t incept_ch6 : 11;
  uint64_t incept_ch7 : 11;
  uint64_t incept_ch8 : 11;
  uint64_t incept_ch9 : 11;
  uint64_t incept_ch10 : 11;
  uint64_t incept_ch11 : 11;
  uint64_t incept_ch12 : 11;
  uint64_t incept_ch13 : 11;
  uint64_t incept_ch14 : 11;
  uint64_t incept_ch15 : 11;
  uint64_t fmu_imu_healthy : 1;
  uint64_t fmu_imu_new_data : 1;
  uint64_t fmu_imu_die_temp_c : 10;
  uint64_t fmu_imu_accel_x_mps2 : 16;
  uint64_t fmu_imu_accel_y_mps2 : 16;
  uint64_t fmu_imu_accel_z_mps2 : 16;
  uint64_t fmu_imu_gyro_x_radps : 16;
  uint64_t fmu_imu_gyro_y_radps : 16;
  uint64_t fmu_imu_gyro_z_radps : 16;
  uint64_t fmu_mag_healthy : 1;
  uint64_t fmu_mag_new_data : 1;
  uint64_t fmu_mag_die_temp_c : 10;
  uint64_t fmu_mag_x_ut : 16;
  uint64_t fmu_mag_y_ut : 16;
  uint64_t fmu_mag_z_ut : 16;
  uint64_t fmu_static_pres_healthy : 1;
  uint64_t fmu_static_pres_new_data : 1;
  uint64_t fmu_static_pres_die_temp_c : 10;
  uint64_t fmu_static_pres_pa : 16;
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  uint64_t vector_nav_installed : 1;
  uint64_t vector_nav_healthy : 1;
  uint64_t vector_nav_die_temp_c : 10;
  uint64_t vector_nav_imu_new_data : 1;
  uint64_t vector_nav_imu_accel_x_mps2 : 16;
  uint64_t vector_nav_imu_accel_y_mps2 : 16;
  uint64_t vector_nav_imu_accel_z_mps2 : 16;
  uint64_t vector_nav_imu_gyro_x_radps : 16;
  uint64_t vector_nav_imu_gyro_y_radps : 16;
  uint64_t vector_nav_imu_gyro_z_radps : 16;
  uint64_t vector_nav_mag_x_ut : 16;
  uint64_t vector_nav_mag_y_ut : 16;
  uint64_t vector_nav_mag_z_ut : 16;
  uint64_t vector_nav_static_pres_pa : 16;
  uint64_t vector_nav_gnss_installed : 1;
  uint64_t vector_nav_gnss_healthy : 1;
  uint64_t vector_nav_gnss_new_data : 1;
  uint64_t vector_nav_gnss_fix : 3;
  uint64_t vector_nav_gnss_num_sats : 5;
  uint64_t vector_nav_gnss_gps_week : 13;
  uint64_t vector_nav_gnss_alt_wgs84 : 16;
  uint64_t vector_nav_gnss_horz_acc_m : 14;
  uint64_t vector_nav_gnss_vert_acc_m : 14;
  uint64_t vector_nav_gnss_vel_acc_mps : 14;
  uint64_t vector_nav_gnss_north_vel_mps : 11;
  uint64_t vector_nav_gnss_east_vel_mps : 11;
  uint64_t vector_nav_gnss_down_vel_mps : 11;
  uint64_t vector_nav_gnss_lat_rad : 32;
  uint64_t vector_nav_gnss_lon_rad : 32;
  uint64_t vector_nav_gnss_gps_tow_s : 30;
  #endif
  uint64_t ext_mag_installed : 1;
  uint64_t ext_mag_healthy : 1;
  uint64_t ext_mag_new_data : 1;
  uint64_t ext_mag_die_temp_c : 10;
  uint64_t ext_mag_x_ut : 16;
  uint64_t ext_mag_y_ut : 16;
  uint64_t ext_mag_z_ut : 16;
  uint64_t ext_gnss1_installed : 1;
  uint64_t ext_gnss1_healthy : 1;
  uint64_t ext_gnss1_new_data : 1;
  uint64_t ext_gnss1_rel_pos_avail : 1;
  uint64_t ext_gnss1_rel_pos_moving_baseline : 1;
  uint64_t ext_gnss1_rel_pos_baseline_normalized : 1;
  uint64_t ext_gnss1_fix : 3;
  uint64_t ext_gnss1_num_sats : 5;
  uint64_t ext_gnss1_gps_week : 13;
  uint64_t ext_gnss1_alt_wgs84_m : 20;
  uint64_t ext_gnss1_horz_acc_m : 14;
  uint64_t ext_gnss1_vert_acc_m : 14;
  uint64_t ext_gnss1_vel_acc_mps : 14;
  uint64_t ext_gnss1_north_vel_mps : 14;
  uint64_t ext_gnss1_east_vel_mps : 14;
  uint64_t ext_gnss1_down_vel_mps : 14;
  uint64_t ext_gnss1_rel_pos_acc_north_m : 16;
  uint64_t ext_gnss1_rel_pos_acc_east_m : 16;
  uint64_t ext_gnss1_rel_pos_acc_down_m : 16;
  uint64_t ext_gnss1_gps_tow_s : 30;
  uint64_t ext_gnss1_lat_rad : 36;
  uint64_t ext_gnss1_lon_rad : 36;
  uint64_t ext_gnss1_rel_pos_north_m : 27;
  uint64_t ext_gnss1_rel_pos_east_m : 27;
  uint64_t ext_gnss1_rel_pos_down_m : 23;
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  uint64_t ext_gnss2_installed : 1;
  uint64_t ext_gnss2_healthy : 1;
  uint64_t ext_gnss2_new_data : 1;
  uint64_t ext_gnss2_rel_pos_avail : 1;
  uint64_t ext_gnss2_rel_pos_moving_baseline : 1;
  uint64_t ext_gnss2_rel_pos_baseline_normalized : 1;
  uint64_t ext_gnss2_fix : 3;
  uint64_t ext_gnss2_num_sats : 5;
  uint64_t ext_gnss2_gps_week : 13;
  uint64_t ext_gnss2_alt_wgs84_m : 20;
  uint64_t ext_gnss2_horz_acc_m : 14;
  uint64_t ext_gnss2_vert_acc_m : 14;
  uint64_t ext_gnss2_vel_acc_mps : 14;
  uint64_t ext_gnss2_north_vel_mps : 14;
  uint64_t ext_gnss2_east_vel_mps : 14;
  uint64_t ext_gnss2_down_vel_mps : 14;
  uint64_t ext_gnss2_rel_pos_acc_north_m : 16;
  uint64_t ext_gnss2_rel_pos_acc_east_m : 16;
  uint64_t ext_gnss2_rel_pos_acc_down_m : 16;
  uint64_t ext_gnss2_gps_tow_s : 30;
  uint64_t ext_gnss2_lat_rad : 36;
  uint64_t ext_gnss2_lon_rad : 36;
  uint64_t ext_gnss2_rel_pos_north_m : 27;
  uint64_t ext_gnss2_rel_pos_east_m : 27;
  uint64_t ext_gnss2_rel_pos_down_m : 23;
  #endif
  uint64_t ext_pres1_installed : 1;
  uint64_t ext_pres1_is_static_pres : 1;
  uint64_t ext_pres1_healthy : 1;
  uint64_t ext_pres1_new_data : 1;
  uint64_t ext_pres1_die_temp_c : 10;
  uint64_t ext_pres1_pres_pa : 16;
  uint64_t ext_pres2_installed : 1;
  uint64_t ext_pres2_is_static_pres : 1;
  uint64_t ext_pres2_healthy : 1;
  uint64_t ext_pres2_new_data : 1;
  uint64_t ext_pres2_die_temp_c : 10;
  uint64_t ext_pres2_pres_pa : 16;
  uint64_t ext_pres3_installed : 1;
  uint64_t ext_pres3_is_static_pres : 1;
  uint64_t ext_pres3_healthy : 1;
  uint64_t ext_pres3_new_data : 1;
  uint64_t ext_pres3_die_temp_c : 10;
  uint64_t ext_pres3_pres_pa : 16;
  uint64_t ext_pres4_installed : 1;
  uint64_t ext_pres4_is_static_pres : 1;
  uint64_t ext_pres4_healthy : 1;
  uint64_t ext_pres4_new_data : 1;
  uint64_t ext_pres4_die_temp_c : 10;
  uint64_t ext_pres4_pres_pa : 16;
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  uint64_t rad_alt_installed : 1;
  uint64_t rad_alt_healthy : 1;
  uint64_t rad_alt_new_data : 1;
  uint64_t rad_alt_snr : 8;
  uint64_t rad_alt_alt_m : 11;
  #endif
  uint64_t ain0_v : 12;
  uint64_t ain1_v : 12;
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  uint64_t ain2_v : 12;
  uint64_t ain3_v : 12;
  uint64_t ain4_v : 12;
  uint64_t ain5_v : 12;
  #endif
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__)
  uint64_t ain6_v : 12;
  uint64_t ain7_v : 12;
  #endif
  #if defined(__FMU_R_V2__) || defined(__FMU_R_MINI_V1__)
  uint64_t power_module_voltage_v : 12;
  uint64_t power_module_current_ma : 18;
  #endif
  uint64_t bfs_ins_initialized : 1;
  uint64_t bfs_ins_pitch_rad : 16;
  uint64_t bfs_ins_roll_rad : 16;
  uint64_t bfs_ins_heading_rad : 16;
  uint64_t bfs_ins_alt_wgs84_m : 20;
  uint64_t bfs_ins_accel_x_mps2 : 16;
  uint64_t bfs_ins_accel_y_mps2 : 16;
  uint64_t bfs_ins_accel_z_mps2 : 16;
  uint64_t bfs_ins_gyro_x_radps : 16;
  uint64_t bfs_ins_gyro_y_radps : 16;
  uint64_t bfs_ins_gyro_z_radps : 16;
  uint64_t bfs_ins_mag_x_ut : 16;
  uint64_t bfs_ins_mag_y_ut : 16;
  uint64_t bfs_ins_mag_z_ut : 16;
  uint64_t bfs_ins_north_vel_mps : 14;
  uint64_t bfs_ins_east_vel_mps : 14;
  uint64_t bfs_ins_down_vel_mps : 14;
  uint64_t bfs_ins_lat_rad : 36;
  uint64_t bfs_ins_lon_rad : 36;
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  uint64_t vector_nav_ins_initialized : 1;
  uint64_t vector_nav_ins_pitch_rad : 16;
  uint64_t vector_nav_ins_roll_rad : 16;
  uint64_t vector_nav_ins_heading_rad : 16;
  uint64_t vector_nav_ins_alt_wgs84_m : 20;
  uint64_t vector_nav_ins_accel_x_mps2 : 16;
  uint64_t vector_nav_ins_accel_y_mps2 : 16;
  uint64_t vector_nav_ins_accel_z_mps2 : 16;
  uint64_t vector_nav_ins_gyro_x_radps : 16;
  uint64_t vector_nav_ins_gyro_y_radps : 16;
  uint64_t vector_nav_ins_gyro_z_radps : 16;
  uint64_t vector_nav_ins_mag_x_ut : 16;
  uint64_t vector_nav_ins_mag_y_ut : 16;
  uint64_t vector_nav_ins_mag_z_ut : 16;
  uint64_t vector_nav_ins_north_vel_mps : 14;
  uint64_t vector_nav_ins_east_vel_mps : 14;
  uint64_t vector_nav_ins_down_vel_mps : 14;
  uint64_t vector_nav_ins_lat_rad : 36;
  uint64_t vector_nav_ins_lon_rad : 36;
  #endif
  uint64_t aux_ins_home_alt_wgs84_m : 20;
  uint64_t aux_ins_gnd_spd_mps : 13;
  uint64_t aux_ins_gnd_track_rad : 16;
  uint64_t aux_ins_flight_path_rad : 15;
  uint64_t aux_ins_home_lat_rad :36;
  uint64_t aux_ins_home_lon_rad : 36;
  uint64_t aux_ins_ned_pos_north_m : 27;
  uint64_t aux_ins_ned_pos_east_m : 27;
  uint64_t aux_ins_ned_pos_down_m : 23;
  uint64_t adc_static_pres_pa : 16;
  uint64_t adc_diff_pres_pa : 16;
  uint64_t adc_pres_alt_m : 16;
  uint64_t adc_rel_alt_m : 16;
  uint64_t adc_ias_mps : 10;
  uint64_t telem_waypoint_frame : 8;
  uint64_t telem_waypoint_cmd : 16;
  uint64_t telem_waypoint_param1 : 32;
  uint64_t telem_waypoint_param2 : 32;
  uint64_t telem_waypoint_param3 : 32;
  uint64_t telem_waypoint_param4 : 32;
  uint64_t telem_waypoint_x : 32;
  uint64_t telem_waypoint_y : 32;
  uint64_t telem_waypoint_z : 32;
  uint64_t telem_param0 : 32;
  uint64_t telem_param1 : 32;
  uint64_t telem_param2 : 32;
  uint64_t telem_param3 : 32;
  uint64_t telem_param4 : 32;
  uint64_t telem_param5 : 32;
  uint64_t telem_param6 : 32;
  uint64_t telem_param7 : 32;
  uint64_t telem_param8 : 32;
  uint64_t telem_param9 : 32;
  uint64_t telem_param10 : 32;
  uint64_t telem_param11 : 32;
  uint64_t telem_param12 : 32;
  uint64_t telem_param13 : 32;
  uint64_t telem_param14 : 32;
  uint64_t telem_param15 : 32;
  uint64_t telem_param16 : 32;
  uint64_t telem_param17 : 32;
  uint64_t telem_param18 : 32;
  uint64_t telem_param19 : 32;
  uint64_t telem_param20 : 32;
  uint64_t telem_param21 : 32;
  uint64_t telem_param22 : 32;
  uint64_t telem_param23 : 32;

  uint64_t vms_advance_waypoint : 1;
  uint64_t vms_motors_enabled : 1;
  uint64_t vms_mode : 3;
  uint64_t vms_sbus_cmd0 : 11;
  uint64_t vms_sbus_cmd1 : 11;
  uint64_t vms_sbus_cmd2 : 11;
  uint64_t vms_sbus_cmd3 : 11;
  uint64_t vms_sbus_cmd4 : 11;
  uint64_t vms_sbus_cmd5 : 11;
  uint64_t vms_sbus_cmd6 : 11;
  uint64_t vms_sbus_cmd7 : 11;
  uint64_t vms_sbus_cmd8 : 11;
  uint64_t vms_sbus_cmd9 : 11;
  uint64_t vms_sbus_cmd10 : 11;
  uint64_t vms_sbus_cmd11 : 11;
  uint64_t vms_sbus_cmd12 : 11;
  uint64_t vms_sbus_cmd13 : 11;
  uint64_t vms_sbus_cmd14 : 11;
  uint64_t vms_sbus_cmd15 : 11;
  uint64_t vms_pwm_cmd0 : 10;
  uint64_t vms_pwm_cmd1 : 10;
  uint64_t vms_pwm_cmd2 : 10;
  uint64_t vms_pwm_cmd3 : 10;
  uint64_t vms_pwm_cmd4 : 10;
  uint64_t vms_pwm_cmd5 : 10;
  uint64_t vms_pwm_cmd6 : 10;
  uint64_t vms_pwm_cmd7 : 10;
  uint64_t vms_flight_time_remaining_s : 16;
  uint64_t vms_power_remaining_prcnt : 10;
  uint64_t vms_aux0 : 32;
  uint64_t vms_aux1 : 32;
  uint64_t vms_aux2 : 32;
  uint64_t vms_aux3 : 32;
  uint64_t vms_aux4 : 32;
  uint64_t vms_aux5 : 32;
  uint64_t vms_aux6 : 32;
  uint64_t vms_aux7 : 32;
  uint64_t vms_aux8 : 32;
  uint64_t vms_aux9 : 32;
  uint64_t vms_aux10 : 32;
  uint64_t vms_aux11 : 32;
  uint64_t vms_aux12 : 32;
  uint64_t vms_aux13 : 32;
  uint64_t vms_aux14 : 32;
  uint64_t vms_aux15 : 32;
  uint64_t vms_aux16 : 32;
  uint64_t vms_aux17 : 32;
  uint64_t vms_aux18 : 32;
  uint64_t vms_aux19 : 32;
  uint64_t vms_aux20 : 32;
  uint64_t vms_aux21 : 32;
  uint64_t vms_aux22 : 32;
  uint64_t vms_aux23 : 32;
};

#endif  // COMMON_DATALOG_FMU_V1_H_
