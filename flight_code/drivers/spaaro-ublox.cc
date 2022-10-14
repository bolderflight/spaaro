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

#include "flight/msg.h"
#include "drivers/spaaro-ublox.h"

void SpaaroUbx::Init(const GnssConfig &cfg) {
  if (cfg.baud > 0) {
    if (!gnss_.Begin(cfg.baud)) {
      MsgError("Unable to establish communication with external GNSS");
    } else {
      installed_ = true;
    }
  } else {
    installed_ = false;
  }
}

void SpaaroUbx::Read(GnssData * const data) {
  data->installed = installed_;
  if (data->installed) {
    data->new_data = gnss_.Read();
    if (data->new_data) {
      t_healthy_ms_ = 0;
      data->rel_pos_avail = gnss_.rel_pos_avail();
      data->rel_pos_moving_baseline = gnss_.rel_pos_moving_baseline();
      data->rel_pos_baseline_normalized = gnss_.rel_pos_normalized();
      data->fix = static_cast<int8_t>(gnss_.fix());
      data->num_sats = gnss_.num_sv();
      data->gps_week = gnss_.gps_week();
      data->alt_wgs84_m = gnss_.alt_wgs84_m();
      data->horz_acc_m = gnss_.horz_acc_m();
      data->vert_acc_m = gnss_.vert_acc_m();
      data->vel_acc_mps = gnss_.spd_acc_mps();
      data->gps_tow_s = gnss_.gps_tow_s();
      data->lat_rad = gnss_.lat_rad();
      data->lon_rad = gnss_.lon_rad();
      data->ned_vel_mps[0] = gnss_.north_vel_mps();
      data->ned_vel_mps[1] = gnss_.east_vel_mps();
      data->ned_vel_mps[2] = gnss_.down_vel_mps();
      data->rel_pos_acc_ned_m[0] = gnss_.rel_pos_acc_north_m();
      data->rel_pos_acc_ned_m[1] = gnss_.rel_pos_acc_east_m();
      data->rel_pos_acc_ned_m[2] = gnss_.rel_pos_acc_down_m();
      data->rel_pos_ned_m[0] = gnss_.rel_pos_north_m();
      data->rel_pos_ned_m[1] = gnss_.rel_pos_east_m();
      data->rel_pos_ned_m[2] = gnss_.rel_pos_down_m();
    }
    data->healthy = (t_healthy_ms_ < 10 * HEALTHY_PERIOD_MS_);
  }
}
