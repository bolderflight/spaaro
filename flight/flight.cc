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
#include "imu/imu.h"
#include "gnss/gnss.h"
#include "static_pres/static_pres.h"
#include "diff_pres/diff_pres.h"
#include "mpu9250_imu/mpu9250_imu.h"
#include "bme280_static_pres/bme280_static_pres.h"
#include "ams5915_static_pres/ams5915_static_pres.h"
#include "ams5915_diff_pres/ams5915_diff_pres.h"

/* Aircraft data */
AircraftData data;
/* IMU */
bfs::Imu<bfs::Mpu9250Imu> imu(&IMU_SPI_BUS, IMU_CS);
/* GNSS */
// bfs::Gnss<> gnss(&GNSS_UART);
/* Airdata */
#if (PITOT_STATIC_INSTALLED)
bfs::StaticPres<
  bfs::Ams5915StaticPres<STATIC_PRES_TYPE>
> static_pres(&PRES_I2C_BUS, STATIC_PRES_ADDR);
bfs::DiffPres<
  bfs::Ams5915DiffPres<DIFF_PRES_TYPE>
> diff_pres(&PRES_I2C_BUS, DIFF_PRES_ADDR);
#else
bfs::StaticPres<
  bfs::Bme280StaticPres
> static_pres(&PRES_SPI_BUS, STATIC_PRES_CS);
#endif
/* Inceptors */

/* Nav */

/* Effectors */

int main() {
  /* Init the message bus */
  MsgBegin();
  /* Init the IMU */
  if (!imu.Init(config.imu)) {MsgError("Unable to communicate with IMU");}
  /* Init the GNSS */
  // if (!gnss.Init(config.gnss)) {MsgError("Unable to communicate with GNSS");}
  /* Init the pressure transducers */
  if (!static_pres.Init(config.static_pres)) {
    MsgError("Unable to communicate with static pressure transducer");
  }
  #if (PITOT_STATIC_INSTALLED)
  if (!diff_pres.Init(config.diff_pres)) {
    MsgError("Unable to communicate with differential pressure transducer");
  }
  #endif
}
