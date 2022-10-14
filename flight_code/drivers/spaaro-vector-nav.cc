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

#if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
    defined(__FMU_R_V2_BETA__)

#include "global_defs.h"
#include "hardware_defs.h"
#include "flight/msg.h"
#include "drivers/spaaro-vector-nav.h"
#include "vector_nav.h"

namespace {
elapsedMillis t_healthy_imu_, t_healthy_gnss_;
static constexpr int16_t HEALTHY_GNSS_PERIOD_MS_ = 1000;
VectorNavConfig cfg_;
bfs::Vn100 vn100_(&SPI_BUS, VN_CS);
bfs::Vn200 vn200_(&SPI_BUS, VN_CS);
bfs::Vn300 vn300_(&SPI_BUS, VN_CS);
double gnss_time_prev_s_;
}  // namespace

void VectorNavInit(const VectorNavConfig & cfg) {
  cfg_ = cfg;
  switch (cfg_.device) {
    case VECTOR_NAV_VN100: {
      if (!vn100_.Begin()) {
        MsgError("Unable to establish communication with VN-100");
      }
      if (!vn100_.SetAccelFilter(bfs::Vn100::FILTER_BOTH, cfg_.accel_filt_window)) {
        MsgError("Unable to configure VN-100 accel filter");
      }
      if (!vn100_.SetGyroFilter(bfs::Vn100::FILTER_BOTH, cfg_.gyro_filt_window)) {
        MsgError("Unable to configure VN-100 gyro filter");
      }
      if (!vn100_.SetMagFilter(bfs::Vn100::FILTER_BOTH, cfg_.mag_filt_window)) {
        MsgError("Unable to configure VN-100 mag filter");
      }
      if (!vn100_.SetTemperatureFilter(bfs::Vn100::FILTER_BOTH, cfg_.temp_filt_window)) {
        MsgError("Unable to configure VN-100 temperature filter");
      }
      if (!vn100_.SetPressureFilter(bfs::Vn100::FILTER_BOTH, cfg_.pres_filt_window)) {
        MsgError("Unable to configure VN-100 pressure filter");
      }
      if (!vn100_.ApplyRotation(cfg_.rotation)) {
        MsgError("Unable to configure VN-100 rotation");
      }
      break;
    }
    case VECTOR_NAV_VN200: {
      if (!vn200_.Begin()) {
        MsgError("Unable to establish communication with VN-200");
      }
      if (!vn200_.SetAccelFilter(bfs::Vn200::FILTER_BOTH, cfg_.accel_filt_window)) {
        MsgError("Unable to configure VN-200 accel filter");
      }
      if (!vn200_.SetGyroFilter(bfs::Vn200::FILTER_BOTH, cfg_.gyro_filt_window)) {
        MsgError("Unable to configure VN-200 gyro filter");
      }
      if (!vn200_.SetMagFilter(bfs::Vn200::FILTER_BOTH, cfg_.mag_filt_window)) {
        MsgError("Unable to configure VN-200 mag filter");
      }
      if (!vn200_.SetTemperatureFilter(bfs::Vn200::FILTER_BOTH, cfg_.temp_filt_window)) {
        MsgError("Unable to configure VN-200 temperature filter");
      }
      if (!vn200_.SetPressureFilter(bfs::Vn200::FILTER_BOTH, cfg_.pres_filt_window)) {
        MsgError("Unable to configure VN-200 pressure filter");
      }
      if (!vn200_.ApplyRotation(cfg_.rotation)) {
        MsgError("Unable to configure VN-200 rotation");
      }
      if (!vn200_.SetAntennaOffset(cfg_.antenna_offset_m)) {
        MsgError("Unable to configure VN-200 antenna offset");
      }
      break;
    }
    case VECTOR_NAV_VN300: {
      if (!vn300_.Begin()) {
        MsgError("Unable to establish communication with VN-300");
      }
      if (!vn300_.SetAccelFilter(bfs::Vn300::FILTER_BOTH, cfg_.accel_filt_window)) {
        MsgError("Unable to configure VN-300 accel filter");
      }
      if (!vn300_.SetGyroFilter(bfs::Vn300::FILTER_BOTH, cfg_.gyro_filt_window)) {
        MsgError("Unable to configure VN-300 gyro filter");
      }
      if (!vn300_.SetMagFilter(bfs::Vn300::FILTER_BOTH, cfg_.mag_filt_window)) {
        MsgError("Unable to configure VN-300 mag filter");
      }
      if (!vn300_.SetTemperatureFilter(bfs::Vn300::FILTER_BOTH, cfg_.temp_filt_window)) {
        MsgError("Unable to configure VN-300 temperature filter");
      }
      if (!vn300_.SetPressureFilter(bfs::Vn300::FILTER_BOTH, cfg_.pres_filt_window)) {
        MsgError("Unable to configure VN-300 pressure filter");
      }
      if (!vn300_.ApplyRotation(cfg_.rotation)) {
        MsgError("Unable to configure VN-300 rotation");
      }
      if (!vn300_.SetAntennaOffset(cfg_.antenna_offset_m)) {
        MsgError("Unable to configure VN-300 antenna offset");
      }
      if (!vn300_.SetCompassBaseline(cfg_.antenna_baseline_m, cfg_.baseline_uncertainty_m)) {
        MsgError("Unable to configure VN-300 compass baseline");
      }
      break;
    }
    default: {
    }
  }
}

void VectorNavRead(ImuData * const imu, MagData * const mag,
                   PresData * const pres, GnssData * const gnss,
                   InsData * const ins) {
  switch (cfg_.device) {
    case VECTOR_NAV_VN100: {
      imu->installed = true;
      mag->installed = true;
      pres->installed = true;
      gnss->installed = false;
      imu->new_data = vn100_.Read();
      mag->new_data = imu->new_data;
      pres->new_data = imu->new_data;
      if(imu->new_data) {
        t_healthy_imu_ = 0;
        imu->die_temp_c = vn100_.die_temp_c();
        mag->die_temp_c = imu->die_temp_c;
        pres->die_temp_c = imu->die_temp_c;
        imu->accel_mps2[0] = vn100_.uncomp_accel_x_mps2();
        imu->accel_mps2[1] = vn100_.uncomp_accel_y_mps2();
        imu->accel_mps2[2] = vn100_.uncomp_accel_z_mps2();
        imu->gyro_radps[0] = vn100_.uncomp_gyro_x_radps();
        imu->gyro_radps[1] = vn100_.uncomp_gyro_y_radps();
        imu->gyro_radps[2] = vn100_.uncomp_gyro_z_radps();
        mag->mag_ut[0] = vn100_.uncomp_mag_x_ut();
        mag->mag_ut[1] = vn100_.uncomp_mag_y_ut();
        mag->mag_ut[2] = vn100_.uncomp_mag_z_ut();
        pres->pres_pa = vn100_.pres_pa();
        ins->initialized = true;
        ins->pitch_rad = vn100_.pitch_rad();
        ins->roll_rad = vn100_.roll_rad();
        ins->heading_rad = vn100_.yaw_rad();
        ins->accel_mps2[0] = vn100_.accel_x_mps2();
        ins->accel_mps2[1] = vn100_.accel_y_mps2();
        ins->accel_mps2[2] = vn100_.accel_z_mps2();
        ins->gyro_radps[0] = vn100_.gyro_x_radps();
        ins->gyro_radps[1] = vn100_.gyro_y_radps();
        ins->gyro_radps[2] = vn100_.gyro_z_radps();
        ins->mag_ut[0] = vn100_.mag_x_ut();
        ins->mag_ut[1] = vn100_.mag_y_ut();
        ins->mag_ut[2] = vn100_.mag_z_ut();
      }
      imu->healthy = (t_healthy_imu_ < 10 * FRAME_PERIOD_MS);
      mag->healthy = (t_healthy_imu_ < 10 * FRAME_PERIOD_MS);
      pres->healthy = (t_healthy_imu_ < 10 * FRAME_PERIOD_MS);
      break;
    }
    case VECTOR_NAV_VN200: {
      imu->installed = true;
      mag->installed = true;
      pres->installed = true;
      gnss->installed = true;
      imu->new_data = vn200_.Read();
      mag->new_data = imu->new_data;
      pres->new_data = imu->new_data;
      if(imu->new_data) {
        t_healthy_imu_ = 0;
        imu->die_temp_c = vn100_.die_temp_c();
        mag->die_temp_c = imu->die_temp_c;
        pres->die_temp_c = imu->die_temp_c;
        imu->accel_mps2[0] = vn200_.uncomp_accel_x_mps2();
        imu->accel_mps2[1] = vn200_.uncomp_accel_y_mps2();
        imu->accel_mps2[2] = vn200_.uncomp_accel_z_mps2();
        imu->gyro_radps[0] = vn200_.uncomp_gyro_x_radps();
        imu->gyro_radps[1] = vn200_.uncomp_gyro_y_radps();
        imu->gyro_radps[2] = vn200_.uncomp_gyro_z_radps();
        mag->mag_ut[0] = vn200_.uncomp_mag_x_ut();
        mag->mag_ut[1] = vn200_.uncomp_mag_y_ut();
        mag->mag_ut[2] = vn200_.uncomp_mag_z_ut();
        pres->pres_pa = vn200_.pres_pa();
        gnss->rel_pos_avail = false;
        if (vn200_.gnss_time_s() != gnss_time_prev_s_) {
          gnss->new_data = true;
          gnss_time_prev_s_ = vn200_.gnss_time_s();
        } else {
          gnss->new_data = false;
        }
        if (gnss->new_data) {
          t_healthy_gnss_ = 0;
          gnss->fix = vn200_.gnss_fix();
          gnss->num_sats = vn200_.gnss_num_sats();
          gnss->gps_week = vn200_.gnss_week();
          gnss->alt_wgs84_m = vn200_.gnss_alt_m();
          gnss->horz_acc_m = (vn200_.gnss_north_acc_m() +
                              vn200_.gnss_east_acc_m()) / 2.0f;
          gnss->vert_acc_m = vn200_.gnss_down_acc_m();
          gnss->vel_acc_mps = vn200_.gnss_speed_acc_mps();
          gnss->ned_vel_mps[0] = vn200_.gnss_north_vel_mps();
          gnss->ned_vel_mps[1] = vn200_.gnss_east_vel_mps();
          gnss->ned_vel_mps[2] = vn200_.gnss_down_vel_mps();
          gnss->gps_tow_s = vn200_.gnss_time_s();
          gnss->lat_rad = vn200_.gnss_lat_rad();
          gnss->lon_rad = vn200_.gnss_lon_rad();
        }
        ins->initialized = (vn200_.ins_mode() == bfs::Vn200::HEALTHY);
        ins->pitch_rad = vn200_.pitch_rad();
        ins->roll_rad = vn200_.roll_rad();
        ins->heading_rad = vn200_.yaw_rad();
        ins->accel_mps2[0] = vn200_.accel_x_mps2();
        ins->accel_mps2[1] = vn200_.accel_y_mps2();
        ins->accel_mps2[2] = vn200_.accel_z_mps2();
        ins->gyro_radps[0] = vn200_.gyro_x_radps();
        ins->gyro_radps[1] = vn200_.gyro_y_radps();
        ins->gyro_radps[2] = vn200_.gyro_z_radps();
        ins->mag_ut[0] = vn200_.mag_x_ut();
        ins->mag_ut[1] = vn200_.mag_y_ut();
        ins->mag_ut[2] = vn200_.mag_z_ut();
        ins->alt_wgs84_m = vn200_.ins_alt_m();
        ins->lat_rad = vn200_.ins_lat_rad();
        ins->lon_rad = vn200_.ins_lon_rad();
        ins->ned_vel_mps[0] = vn200_.ins_north_vel_mps();
        ins->ned_vel_mps[1] = vn200_.ins_east_vel_mps();
        ins->ned_vel_mps[2] = vn200_.ins_down_vel_mps();
      }
      imu->healthy = (t_healthy_imu_ < 10 * FRAME_PERIOD_MS);
      mag->healthy = (t_healthy_imu_ < 10 * FRAME_PERIOD_MS);
      pres->healthy = (t_healthy_imu_ < 10 * FRAME_PERIOD_MS);
      gnss->healthy = (t_healthy_gnss_ < 10 * HEALTHY_GNSS_PERIOD_MS_);
      break;
    }
    case VECTOR_NAV_VN300: {
      imu->installed = true;
      mag->installed = true;
      pres->installed = true;
      gnss->installed = true;
      imu->new_data = vn300_.Read();
      mag->new_data = imu->new_data;
      pres->new_data = imu->new_data;
      if(imu->new_data) {
        t_healthy_gnss_ = 0;
        imu->die_temp_c = vn100_.die_temp_c();
        mag->die_temp_c = imu->die_temp_c;
        pres->die_temp_c = imu->die_temp_c;
        imu->accel_mps2[0] = vn300_.uncomp_accel_x_mps2();
        imu->accel_mps2[1] = vn300_.uncomp_accel_y_mps2();
        imu->accel_mps2[2] = vn300_.uncomp_accel_z_mps2();
        imu->gyro_radps[0] = vn300_.uncomp_gyro_x_radps();
        imu->gyro_radps[1] = vn300_.uncomp_gyro_y_radps();
        imu->gyro_radps[2] = vn300_.uncomp_gyro_z_radps();
        mag->mag_ut[0] = vn300_.uncomp_mag_x_ut();
        mag->mag_ut[1] = vn300_.uncomp_mag_y_ut();
        mag->mag_ut[2] = vn300_.uncomp_mag_z_ut();
        pres->pres_pa = vn300_.pres_pa();
        gnss->rel_pos_avail = false;
        if (vn300_.gnss_time_s() != gnss_time_prev_s_) {
          gnss->new_data = true;
          gnss_time_prev_s_ = vn300_.gnss_time_s();
        } else {
          gnss->new_data = false;
        }
        if (gnss->new_data) {
          t_healthy_gnss_ = 0;
          gnss->fix = vn300_.gnss_fix();
          gnss->num_sats = vn300_.gnss_num_sats();
          gnss->gps_week = vn300_.gnss_week();
          gnss->alt_wgs84_m = vn300_.gnss_alt_m();
          gnss->horz_acc_m = (vn300_.gnss_north_acc_m() +
                              vn300_.gnss_east_acc_m()) / 2.0f;
          gnss->vert_acc_m = vn300_.gnss_down_acc_m();
          gnss->vel_acc_mps = vn300_.gnss_speed_acc_mps();
          gnss->ned_vel_mps[0] = vn300_.gnss_north_vel_mps();
          gnss->ned_vel_mps[1] = vn300_.gnss_east_vel_mps();
          gnss->ned_vel_mps[2] = vn300_.gnss_down_vel_mps();
          gnss->gps_tow_s = vn300_.gnss_time_s();
          gnss->lat_rad = vn300_.gnss_lat_rad();
          gnss->lon_rad = vn300_.gnss_lon_rad();
        }
        ins->initialized = (vn300_.ins_mode() == bfs::Vn300::HEALTHY);
        ins->pitch_rad = vn300_.pitch_rad();
        ins->roll_rad = vn300_.roll_rad();
        ins->heading_rad = vn300_.yaw_rad();
        ins->accel_mps2[0] = vn300_.accel_x_mps2();
        ins->accel_mps2[1] = vn300_.accel_y_mps2();
        ins->accel_mps2[2] = vn300_.accel_z_mps2();
        ins->gyro_radps[0] = vn300_.gyro_x_radps();
        ins->gyro_radps[1] = vn300_.gyro_y_radps();
        ins->gyro_radps[2] = vn300_.gyro_z_radps();
        ins->mag_ut[0] = vn300_.mag_x_ut();
        ins->mag_ut[1] = vn300_.mag_y_ut();
        ins->mag_ut[2] = vn300_.mag_z_ut();
        ins->alt_wgs84_m = vn300_.ins_alt_m();
        ins->lat_rad = vn300_.ins_lat_rad();
        ins->lon_rad = vn300_.ins_lon_rad();
        ins->ned_vel_mps[0] = vn300_.ins_north_vel_mps();
        ins->ned_vel_mps[1] = vn300_.ins_east_vel_mps();
        ins->ned_vel_mps[2] = vn300_.ins_down_vel_mps();
      }
      imu->healthy = (t_healthy_imu_ < 10 * FRAME_PERIOD_MS);
      mag->healthy = (t_healthy_imu_ < 10 * FRAME_PERIOD_MS);
      pres->healthy = (t_healthy_imu_ < 10 * FRAME_PERIOD_MS);
      gnss->healthy = (t_healthy_gnss_ < 10 * HEALTHY_GNSS_PERIOD_MS_);
      break;
    }
    default: {
      imu->installed = false;
      mag->installed = false;
      pres->installed = false;
      gnss->installed = false;
      ins->initialized = false;
    }
  }
}

#endif
