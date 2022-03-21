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

#include "flight/ubx_impl.h"
#include "flight/global_defs.h"
#include "flight/hardware_defs.h"
#include "flight/msg.h"
#include "ubx.h"

namespace {
/* uBlox config */
GnssConfig cfg_gnss3, cfg_gnss4;
/* uBlox objects */
Ubx gnss3(&Serial3), gnss4(&Serial4);
/* Status */
bool status3, status4;
/* Healthy */
bool healthy3, healthy4;
elapsedMillis t3_ms, t4_ms;
inline constexpr int32_t GNSS_TIMEOUT_MS = 5000;
}

void UbxInit(const GnssConfig &g3, const GnssConfig &g4) {
  cfg_gnss3 = g3;
  cfg_gnss4 = g4;
  if (cfg_gnss3.baud > 0) {
    if (!gnss3.Begin(cfg_gnss3.baud)) {
      MsgError("unable to establish communication with UART3 uBlox");
    }
  }
  if (cfg_gnss4.baud > 0) {
    if (!gnss4.Begin(cfg_gnss4.baud)) {
      MsgError("unable to establish communication with UART4 uBlox");
    }
  }
}

void UbxRead() {
  if (cfg_gnss3.baud > 0) {
    status3 = gnss3.Read();
    if (status3) {
      t3_ms = 0;
    }
    healthy3 = (t3_ms < GNSS_TIMEOUT_MS);
  }
  if (cfg_gnss4.baud > 0) {
    status4 = gnss4.Read();
    if (status4) {
      t4_ms = 0;
    }
    healthy4 = (t4_ms < GNSS_TIMEOUT_MS);
  }
}

void UbxGnssData(GnssData * const g3, GnssRelPosData * const rp3,
                 GnssData * const g4, GnssRelPosData * const rp4) {
  if (cfg_gnss3.baud > 0) {
    g3->installed = true;
    g3->healthy = healthy3;
    g3->new_data = status3;
    if (g3->new_data) {
      g3->fix = gnss3.fix();
      g3->num_sats = gnss3.num_sv();
      g3->gps_week = gnss3.gps_week();
      g3->alt_wgs84_m = gnss3.alt_wgs84_m();
      g3->horz_acc_m = gnss3.horz_acc_m();
      g3->vert_acc_m = gnss3.vert_acc_m();
      g3->vel_acc_mps = gnss3.spd_acc_mps();
      g3->ned_vel_mps[0] = gnss3.north_vel_mps();
      g3->ned_vel_mps[1] = gnss3.east_vel_mps();
      g3->ned_vel_mps[2] = gnss3.down_vel_mps();
      g3->gps_tow_s = gnss3.gps_tow_s();
      g3->lat_rad = gnss3.lat_rad();
      g3->lon_rad = gnss3.lon_rad();
      rp3->avail = gnss3.rel_pos_avail();
      rp3->moving_baseline = gnss3.rel_pos_moving_baseline();
      rp3->heading_valid = gnss3.rel_pos_heading_valid();
      rp3->baseline_normalized = gnss3.rel_pos_normalized();
      rp3->baseline_len_acc_m = gnss3.rel_pos_len_acc_m();
      rp3->heading_rad = gnss3.rel_pos_heading_rad();
      rp3->heading_acc_rad = gnss3.rel_pos_heading_acc_rad();
      rp3->baseline_len_m = gnss3.rel_pos_len_m();
      rp3->rel_pos_acc_ned_m[0] = gnss3.rel_pos_acc_north_m();
      rp3->rel_pos_acc_ned_m[1] = gnss3.rel_pos_acc_east_m();
      rp3->rel_pos_acc_ned_m[2] = gnss3.rel_pos_acc_down_m();
      rp3->rel_pos_ned_m[0] = gnss3.rel_pos_north_m();
      rp3->rel_pos_ned_m[1] = gnss3.rel_pos_east_m();
      rp3->rel_pos_ned_m[2] = gnss3.rel_pos_down_m();
    }
  } else {
    g3->installed = false;
    rp3->avail = false;
  }
  if (cfg_gnss4.baud > 0) {
    g4->installed = true;
    g4->healthy = healthy3;
    g4->new_data = status3;
    if (g4->new_data) {
      g4->fix = gnss4.fix();
      g4->num_sats = gnss4.num_sv();
      g4->gps_week = gnss4.gps_week();
      g4->alt_wgs84_m = gnss4.alt_wgs84_m();
      g4->horz_acc_m = gnss4.horz_acc_m();
      g4->vert_acc_m = gnss4.vert_acc_m();
      g4->vel_acc_mps = gnss4.spd_acc_mps();
      g4->ned_vel_mps[0] = gnss4.north_vel_mps();
      g4->ned_vel_mps[1] = gnss4.east_vel_mps();
      g4->ned_vel_mps[2] = gnss4.down_vel_mps();
      g4->gps_tow_s = gnss4.gps_tow_s();
      g4->lat_rad = gnss4.lat_rad();
      g4->lon_rad = gnss4.lon_rad();
      rp4->avail = gnss4.rel_pos_avail();
      rp4->moving_baseline = gnss4.rel_pos_moving_baseline();
      rp4->heading_valid = gnss4.rel_pos_heading_valid();
      rp4->baseline_normalized = gnss4.rel_pos_normalized();
      rp4->baseline_len_acc_m = gnss4.rel_pos_len_acc_m();
      rp4->heading_rad = gnss4.rel_pos_heading_rad();
      rp4->heading_acc_rad = gnss4.rel_pos_heading_acc_rad();
      rp4->baseline_len_m = gnss4.rel_pos_len_m();
      rp4->rel_pos_acc_ned_m[0] = gnss4.rel_pos_acc_north_m();
      rp4->rel_pos_acc_ned_m[1] = gnss4.rel_pos_acc_east_m();
      rp4->rel_pos_acc_ned_m[2] = gnss4.rel_pos_acc_down_m();
      rp4->rel_pos_ned_m[0] = gnss4.rel_pos_north_m();
      rp4->rel_pos_ned_m[1] = gnss4.rel_pos_east_m();
      rp4->rel_pos_ned_m[2] = gnss4.rel_pos_down_m();
    }
  } else {
    g4->installed = false;
    rp4->avail = false;
  }
}
