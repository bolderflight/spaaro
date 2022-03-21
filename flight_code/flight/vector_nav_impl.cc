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

#include "flight/vector_nav_impl.h"
#include "flight/global_defs.h"
#include "flight/hardware_defs.h"
#include "flight/msg.h"
#include "vector_nav.h"  // NOLINT

namespace {
/* VectorNav objects */
Vn100 vn100(&SPI_BUS, VN_CS);
Vn200 vn200(&SPI_BUS, VN_CS);
Vn300 vn300(&SPI_BUS, VN_CS);
/* VectorNav config */
VnConfig config;
/* To determine if GNSS has new data */
double prev_gnss_tow = 0;
inline constexpr int32_t GNSS_TIMEOUT_MS = 5000;
elapsedMillis time_gnss;
/* Interim data */
bool status, healthy;
elapsedMillis time_ms;
}  // namespace

void VectorNavInit(const VnConfig &cfg) {
  config = cfg;
  switch (config.device) {
    case VECTORNAV_VN100: {
      if (!vn100.Begin()) {
        MsgError("unable to establish communication with VN-100");
      }
      if (!vn100.ApplyRotation(config.rotation)) {
        MsgError("unable to set VN-100 rotation");
      }
      if (!vn100.SetMagFilter(Vn100::FILTER_BOTH, config.mag_filt_window)) {
        MsgError("unable to set VN-100 mag filter window");
      }
      if (!vn100.SetAccelFilter(Vn100::FILTER_BOTH,
                                config.accel_filt_window)) {
        MsgError("unable to set VN-100 accel filter window");
      }
      if (!vn100.SetGyroFilter(Vn100::FILTER_BOTH, config.gyro_filt_window)) {
        MsgError("unable to set VN-100 gyro filter window");
      }
      if (!vn100.SetTemperatureFilter(Vn100::FILTER_BOTH,
                                      config.temp_filt_window)) {
        MsgError("unable to set VN-100 temperature filter window");
      }
      if (!vn100.SetPressureFilter(Vn100::FILTER_BOTH,
                                   config.pres_filt_window)) {
        MsgError("unable to set VN-100 pressure filter window");
      }
      break;
    }
    case VECTORNAV_VN200: {
      if (!vn200.Begin()) {
        MsgError("unable to establish communication with VN-200");
      }
      if (!vn200.ApplyRotation(config.rotation)) {
        MsgError("unable to set VN-200 rotation");
      }
      if (!vn200.SetMagFilter(Vn200::FILTER_BOTH, config.mag_filt_window)) {
        MsgError("unable to set VN-200 mag filter window");
      }
      if (!vn200.SetAccelFilter(Vn200::FILTER_BOTH,
                                config.accel_filt_window)) {
        MsgError("unable to set VN-200 accel filter window");
      }
      if (!vn200.SetGyroFilter(Vn200::FILTER_BOTH, config.gyro_filt_window)) {
        MsgError("unable to set VN-200 gyro filter window");
      }
      if (!vn200.SetTemperatureFilter(Vn200::FILTER_BOTH,
                                      config.temp_filt_window)) {
        MsgError("unable to set VN-200 temperature filter window");
      }
      if (!vn200.SetPressureFilter(Vn200::FILTER_BOTH,
                                   config.pres_filt_window)) {
        MsgError("unable to set VN-200 pressure filter window");
      }
      if (!vn200.SetAntennaOffset(config.antenna_offset_m)) {
        MsgError("unable to set VN-200 antenna offset");
      }
      break;
    }
    case VECTORNAV_VN300: {
      if (!vn300.Begin()) {
        MsgError("unable to establish communication with VN-300");
      }
      if (!vn300.ApplyRotation(config.rotation)) {
        MsgError("unable to set VN-300 rotation");
      }
      if (!vn300.SetMagFilter(Vn300::FILTER_BOTH, config.mag_filt_window)) {
        MsgError("unable to set VN-300 mag filter window");
      }
      if (!vn300.SetAccelFilter(Vn300::FILTER_BOTH,
                                config.accel_filt_window)) {
        MsgError("unable to set VN-300 accel filter window");
      }
      if (!vn300.SetGyroFilter(Vn300::FILTER_BOTH, config.gyro_filt_window)) {
        MsgError("unable to set VN-300 gyro filter window");
      }
      if (!vn300.SetTemperatureFilter(Vn300::FILTER_BOTH,
                                      config.temp_filt_window)) {
        MsgError("unable to set VN-300 temperature filter window");
      }
      if (!vn300.SetPressureFilter(Vn300::FILTER_BOTH,
                                   config.pres_filt_window)) {
        MsgError("unable to set VN-300 pressure filter window");
      }
      if (!vn300.SetAntennaOffset(config.antenna_offset_m)) {
        MsgError("unable to set VN-300 antenna offset");
      }
      if (!vn300.SetCompassBaseline(config.antenna_baseline_m,
                                    config.baseline_uncertainty_m)) {
        MsgError("unable to set VN-300 compass baseline");
      }
      break;
    }
  }
}

void VectorNavEnableDrdy() {
  switch (config.device) {
    case VECTORNAV_VN100: {
      if (!vn100.EnableDrdyInt(Vn100::AHRS, VN_SRD)) {
        MsgError("unable to enable VN-100 DRDY interrupt");
      }
      break;
    }
    case VECTORNAV_VN200: {
      if (!vn200.EnableDrdyInt(Vn200::INS, VN_SRD)) {
        MsgError("unable to enable VN-200 DRDY interrupt");
      }
      break;
    }
    case VECTORNAV_VN300: {
      if (!vn300.EnableDrdyInt(Vn300::INS, VN_SRD)) {
        MsgError("unable to enable VN-300 DRDY interrupt");
      }
      break;
    }
  }
}

void VectorNavRead() {
  switch (config.device) {
    case VECTORNAV_VN100: {
      status = vn100.Read();
      if (status) {
        time_ms = 0;
      } else {
        MsgWarning("error reading VN-100");
      }
      healthy = (time_ms < HEALTHY_TIMEOUT_MS);
      break;
    }
    case VECTORNAV_VN200: {
      status = vn200.Read();
      if (status) {
        time_ms = 0;
      } else {
        MsgWarning("error reading VN-200");
      }
      healthy = (time_ms < HEALTHY_TIMEOUT_MS);
      break;
    }
    case VECTORNAV_VN300: {
      status = vn300.Read();
      if (status) {
        time_ms = 0;
      } else {
        MsgWarning("error reading VN-300");
      }
      healthy = (time_ms < HEALTHY_TIMEOUT_MS);
      break;
    }
  }
}

void VectorNavImuData(ImuData * const imu) {
  if (!imu) {return;}
  switch (config.device) {
    case VECTORNAV_NONE: {
      imu->installed = false;
      break;
    }
    case VECTORNAV_VN100: {
      imu->installed = true;
      imu->healthy = healthy;
      imu->new_imu_data = status;
      imu->new_mag_data = status;
      if (imu->new_imu_data) {
        imu->die_temp_c = vn100.die_temp_c();
        imu->accel_mps2[0] = vn100.uncomp_accel_x_mps2();
        imu->accel_mps2[1] = vn100.uncomp_accel_y_mps2();
        imu->accel_mps2[2] = vn100.uncomp_accel_z_mps2();
        imu->gyro_radps[0] = vn100.uncomp_gyro_x_radps();
        imu->gyro_radps[1] = vn100.uncomp_gyro_y_radps();
        imu->gyro_radps[2] = vn100.uncomp_gyro_z_radps();
        imu->mag_ut[0] = vn100.uncomp_mag_x_ut();
        imu->mag_ut[1] = vn100.uncomp_mag_y_ut();
        imu->mag_ut[2] = vn100.uncomp_mag_z_ut();
      }
      break;
    }
    case VECTORNAV_VN200: {
      imu->installed = true;
      imu->healthy = healthy;
      imu->new_imu_data = status;
      imu->new_mag_data = status;
      if (imu->new_imu_data) {
        imu->die_temp_c = vn200.die_temp_c();
        imu->accel_mps2[0] = vn200.uncomp_accel_x_mps2();
        imu->accel_mps2[1] = vn200.uncomp_accel_y_mps2();
        imu->accel_mps2[2] = vn200.uncomp_accel_z_mps2();
        imu->gyro_radps[0] = vn200.uncomp_gyro_x_radps();
        imu->gyro_radps[1] = vn200.uncomp_gyro_y_radps();
        imu->gyro_radps[2] = vn200.uncomp_gyro_z_radps();
        imu->mag_ut[0] = vn200.uncomp_mag_x_ut();
        imu->mag_ut[1] = vn200.uncomp_mag_y_ut();
        imu->mag_ut[2] = vn200.uncomp_mag_z_ut();
      }
      break;
    }
    case VECTORNAV_VN300: {
      imu->installed = true;
      imu->healthy = healthy;
      imu->new_imu_data = status;
      imu->new_mag_data = status;
      if (imu->new_imu_data) {
        imu->die_temp_c = vn300.die_temp_c();
        imu->accel_mps2[0] = vn300.uncomp_accel_x_mps2();
        imu->accel_mps2[1] = vn300.uncomp_accel_y_mps2();
        imu->accel_mps2[2] = vn300.uncomp_accel_z_mps2();
        imu->gyro_radps[0] = vn300.uncomp_gyro_x_radps();
        imu->gyro_radps[1] = vn300.uncomp_gyro_y_radps();
        imu->gyro_radps[2] = vn300.uncomp_gyro_z_radps();
        imu->mag_ut[0] = vn300.uncomp_mag_x_ut();
        imu->mag_ut[1] = vn300.uncomp_mag_y_ut();
        imu->mag_ut[2] = vn300.uncomp_mag_z_ut();
      }
      break;
    }
  }
}

void VectorNavPresData(PresData * const pres) {
  if (!pres) {return;}
  switch (config.device) {
    case VECTORNAV_NONE: {
      pres->installed = false;
      break;
    }
    case VECTORNAV_VN100: {
      pres->installed = true;
      pres->healthy = healthy;
      pres->new_data = status;
      if (pres->new_data) {
        pres->die_temp_c = vn100.die_temp_c();
        pres->pres_pa = vn100.pres_pa();
      }
      break;
    }
    case VECTORNAV_VN200: {
      pres->installed = true;
      pres->healthy = healthy;
      pres->new_data = status;
      if (pres->new_data) {
        pres->die_temp_c = vn200.die_temp_c();
        pres->pres_pa = vn200.pres_pa();
      }
      break;
    }
    case VECTORNAV_VN300: {
      pres->installed = true;
      pres->healthy = healthy;
      pres->new_data = status;
      if (pres->new_data) {
        pres->die_temp_c = vn300.die_temp_c();
        pres->pres_pa = vn300.pres_pa();
      }
      break;
    }
  }
}

void VectorNavGnssData(GnssData * const gnss) {
  if (!gnss) {return;}
  switch (config.device) {
    case VECTORNAV_NONE: {
      gnss->installed = false;
      break;
    }
    case VECTORNAV_VN100: {
      gnss->installed = false;
      break;
    }
    case VECTORNAV_VN200: {
      gnss->installed = true;
      if (vn200.gnss_time_s() != prev_gnss_tow) {
        time_gnss = 0;
        prev_gnss_tow = vn200.gnss_time_s();
        gnss->new_data = true;
        gnss->fix = vn200.gnss_fix();
        gnss->num_sats = vn200.gnss_num_sats();
        gnss->gps_week = vn200.gnss_week();
        gnss->alt_wgs84_m = vn200.gnss_alt_m();
        gnss->horz_acc_m = std::sqrt(vn200.gnss_north_acc_m() *
                                     vn200.gnss_north_acc_m() +
                                     vn200.gnss_east_acc_m() *
                                     vn200.gnss_east_acc_m());
        gnss->vert_acc_m = vn200.gnss_down_acc_m();
        gnss->vel_acc_mps = vn200.gnss_speed_acc_mps();
        gnss->ned_vel_mps[0] = vn200.gnss_north_vel_mps();
        gnss->ned_vel_mps[1] = vn200.gnss_east_vel_mps();
        gnss->ned_vel_mps[2] = vn200.gnss_down_vel_mps();
        gnss->gps_tow_s = vn200.gnss_time_s();
        gnss->lat_rad = vn200.gnss_lat_rad();
        gnss->lon_rad = vn200.gnss_lon_rad();
      } else {
        gnss->new_data = false;
      }
      gnss->healthy = (time_gnss < GNSS_TIMEOUT_MS);
      break;
    }
    case VECTORNAV_VN300: {
      gnss->installed = true;
      if (vn300.gnss_time_s() != prev_gnss_tow) {
        time_gnss = 0;
        prev_gnss_tow = vn300.gnss_time_s();
        gnss->new_data = true;
        gnss->fix = vn300.gnss_fix();
        gnss->num_sats = vn300.gnss_num_sats();
        gnss->gps_week = vn300.gnss_week();
        gnss->alt_wgs84_m = vn300.gnss_alt_m();
        gnss->horz_acc_m = std::sqrt(vn300.gnss_north_acc_m() *
                                     vn300.gnss_north_acc_m() +
                                     vn300.gnss_east_acc_m() *
                                     vn300.gnss_east_acc_m());
        gnss->vert_acc_m = vn300.gnss_down_acc_m();
        gnss->vel_acc_mps = vn300.gnss_speed_acc_mps();
        gnss->ned_vel_mps[0] = vn300.gnss_north_vel_mps();
        gnss->ned_vel_mps[1] = vn300.gnss_east_vel_mps();
        gnss->ned_vel_mps[2] = vn300.gnss_down_vel_mps();
        gnss->gps_tow_s = vn300.gnss_time_s();
        gnss->lat_rad = vn300.gnss_lat_rad();
        gnss->lon_rad = vn300.gnss_lon_rad();
      } else {
        gnss->new_data = false;
      }
      gnss->healthy = (time_gnss < GNSS_TIMEOUT_MS);
      break;
    }
  }
}

void VectorNavInsData(InertialData * const ins) {
  if (!ins) {return;}
  switch (config.device) {
    case VECTORNAV_VN100: {
      ins->healthy = true;
      ins->pitch_rad = vn100.pitch_rad();
      ins->roll_rad = vn100.roll_rad();
      ins->heading_rad = vn100.yaw_rad();
      ins->accel_mps2[0] = vn100.accel_x_mps2();
      ins->accel_mps2[1] = vn100.accel_y_mps2();
      ins->accel_mps2[2] = vn100.accel_z_mps2();
      ins->gyro_radps[0] = vn100.gyro_x_radps();
      ins->gyro_radps[1] = vn100.gyro_y_radps();
      ins->gyro_radps[2] = vn100.gyro_z_radps();
      ins->mag_ut[0] = vn100.mag_x_ut();
      ins->mag_ut[1] = vn100.mag_y_ut();
      ins->mag_ut[2] = vn100.mag_z_ut();
      break;
    }
    case VECTORNAV_VN200: {
      ins->healthy = (vn200.ins_mode() == Vn200::HEALTHY);
      ins->pitch_rad = vn200.pitch_rad();
      ins->roll_rad = vn200.roll_rad();
      ins->heading_rad = vn200.yaw_rad();
      ins->accel_mps2[0] = vn200.accel_x_mps2();
      ins->accel_mps2[1] = vn200.accel_y_mps2();
      ins->accel_mps2[2] = vn200.accel_z_mps2();
      ins->gyro_radps[0] = vn200.gyro_x_radps();
      ins->gyro_radps[1] = vn200.gyro_y_radps();
      ins->gyro_radps[2] = vn200.gyro_z_radps();
      ins->mag_ut[0] = vn200.mag_x_ut();
      ins->mag_ut[1] = vn200.mag_y_ut();
      ins->mag_ut[2] = vn200.mag_z_ut();
      ins->ned_vel_mps[0] = vn200.ins_north_vel_mps();
      ins->ned_vel_mps[1] = vn200.ins_east_vel_mps();
      ins->ned_vel_mps[2] = vn200.ins_down_vel_mps();
      ins->lat_rad = vn200.ins_lat_rad();
      ins->lon_rad = vn200.ins_lon_rad();
      ins->alt_wgs84_m = vn200.ins_alt_m();
      break;
    }
    case VECTORNAV_VN300: {
      ins->healthy = (vn300.ins_mode() == Vn300::HEALTHY);
      ins->pitch_rad = vn300.pitch_rad();
      ins->roll_rad = vn300.roll_rad();
      ins->heading_rad = vn300.yaw_rad();
      ins->accel_mps2[0] = vn300.accel_x_mps2();
      ins->accel_mps2[1] = vn300.accel_y_mps2();
      ins->accel_mps2[2] = vn300.accel_z_mps2();
      ins->gyro_radps[0] = vn300.gyro_x_radps();
      ins->gyro_radps[1] = vn300.gyro_y_radps();
      ins->gyro_radps[2] = vn300.gyro_z_radps();
      ins->mag_ut[0] = vn300.mag_x_ut();
      ins->mag_ut[1] = vn300.mag_y_ut();
      ins->mag_ut[2] = vn300.mag_z_ut();
      ins->ned_vel_mps[0] = vn300.ins_north_vel_mps();
      ins->ned_vel_mps[1] = vn300.ins_east_vel_mps();
      ins->ned_vel_mps[2] = vn300.ins_down_vel_mps();
      ins->lat_rad = vn300.ins_lat_rad();
      ins->lon_rad = vn300.ins_lon_rad();
      ins->alt_wgs84_m = vn300.ins_alt_m();
      break;
    }
  }
}
