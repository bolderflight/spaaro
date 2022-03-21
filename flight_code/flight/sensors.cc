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

#include "flight/sensors.h"
#include "flight/global_defs.h"
#include "flight/hardware_defs.h"
#include "flight/msg.h"
#include "flight/mpu9250_impl.h"
#include "flight/bme280_impl.h"
#include "flight/vector_nav_impl.h"
#include "flight/ams5915_impl.h"
#include "flight/sbus_rx_impl.h"
#include "flight/ubx_impl.h"
#include "flight/analog.h"
#if defined(__FMU_R_V2__)
#include "flight/battery.h"
#endif

void InitSensors(const SensorConfig &cfg) {
  MsgInfo("Initializing sensors...");
  /* Initialize the SBUS inceptors */
  SbusRxInit(cfg.sbus);
  /* Initialize the MPU-9250 */
  Mpu9250Init(cfg.mpu9250);
  /* Initialize the BME280 */
  Bme280Init();
  /* Initialize the VectorNav */
  VectorNavInit(cfg.vector_nav);
  /* Initialize the AMS5915 transducers */
  Ams5915Init(cfg.ams5915_static_pres, cfg.ams5915_diff_pres);
  /* Initialize the uBlox receivers */
  UbxInit(cfg.gnss_uart3, cfg.gnss_uart4);
  /* Enable the data ready source */
  if (cfg.drdy_source == DRDY_MPU9250) {
    Mpu9250EnableDrdy();
  } else {
    if (cfg.vector_nav.device != VECTORNAV_NONE) {
      VectorNavEnableDrdy();
    } else {
      MsgError("VectorNav cannot support DRDY since it's not configured");
    }
  }
  MsgInfo("done.\n");
}

void ReadSensors(SensorData * const data) {
  if (!data) {return;}
  /* Read the SBUS RX */
  SbusRxRead();
  SbusRxInceptorData(&data->sbus_inceptor);
  /* Read the MPU-9250 */
  Mpu9250Read();
  Mpu9250ImuData(&data->mpu9250_imu);
  /* Read the BME280 */
  Bme280Read();
  Bme280PresData(&data->bme280_static_pres);
  /* Read the VectorNav */
  VectorNavRead();
  VectorNavImuData(&data->vector_nav_imu);
  VectorNavPresData(&data->vector_nav_static_pres);
  VectorNavGnssData(&data->vector_nav_gnss);
  /* Read the AMS5915 */
  Ams5915Read();
  Ams5915PresData(&data->ams5915_static_pres,
                  &data->ams5915_diff_pres);
  /* Read the uBlox */
  UbxRead();
  UbxGnssData(&data->ublox3_gnss, &data->ublox3_relpos,
              &data->ublox4_gnss, &data->ublox4_relpos);
  /* Analog to digital converters */
  AnalogRead(&data->adc);
  /* Power module */
  #if defined(__FMU_R_V2__)
  BatteryRead(&data->power_module);
  #endif
}
