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
#include "flight/gnss.h"
#include "ublox/ublox.h"

namespace {
bfs::Ublox gnss_(&GNSS_UART);
/* Year, Month, Day to GNSS week */
int GnssWeek(int year, int month, int day) {
  static const int month_day[2][12] = {
    {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334},
    {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}
  };
  static const int JAN61980 = 44244;
  static const int JAN11901 = 15385;
  int yday, mjd, leap;
  leap = (year % 4 == 0);
  yday = month_day[leap][month - 1] + day;
  mjd = ((year - 1901) / 4) * 1461 + ((year - 1901) % 4) * 365 + yday - 1 +
        JAN11901;
  return (mjd - JAN61980) / 7;
}
}  // namespace

void GnssInit() {
  MsgInfo("Initializing gnss...");
  while (!gnss_.Begin(GNSS_BAUD)) {}
  MsgInfo("done.\n");
}
bool GnssRead(GnssData * const ptr) {
  if (!ptr) {return false;}
  ptr->new_data = gnss_.Read();
  if (ptr->new_data) {
    ptr->tow_s = static_cast<double>(gnss_.tow_ms()) / 1e3;
    ptr->week = GnssWeek(gnss_.year(), gnss_.month(), gnss_.day());
    switch (gnss_.fix()) {
      case bfs::Ublox::FIX_2D: {
        ptr->fix = 2;
        break;
      }
      case bfs::Ublox::FIX_3D: {
        ptr->fix = 3;
        break;
      }
      default: {
        ptr->fix = 0;
        break;
      }
    }
    ptr->num_sats = gnss_.num_satellites();
    ptr->lat_rad = gnss_.lat_rad();
    ptr->lon_rad = gnss_.lon_rad();
    ptr->alt_wgs84_m = gnss_.alt_wgs84_m();
    ptr->north_vel_mps = gnss_.north_velocity_mps();
    ptr->east_vel_mps = gnss_.east_velocity_mps();
    ptr->down_vel_mps = gnss_.down_velocity_mps();
    ptr->horz_acc_m = gnss_.horizontal_accuracy_m();
    ptr->vert_acc_m = gnss_.vertical_accuracy_m();
    ptr->vel_acc_mps = gnss_.velocity_accuracy_mps();
    return true;
  }
  return false;
}
