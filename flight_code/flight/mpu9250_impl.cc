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

#include "flight/mpu9250_impl.h"
#include "flight/global_defs.h"
#include "flight/hardware_defs.h"
#include "flight/msg.h"
#include "mpu9250.h"
#include "statistics.h"

namespace {
/* MPU-9250 object */
Mpu9250 mpu9250(&SPI_BUS, MPU9250_CS);
/* Copy of the configuration */
Mpu9250Config config;
/* Gyro bias estimation */
elapsedMillis time_ms;
static constexpr int32_t INIT_TIME_MS = 5000;
RunningStats<float> gx, gy, gz;
Vector3f gyro_bias_radps;
/* Intermediate results */
Vector3f accel_mps2, gyro_radps, mag_ut;
bool status, healthy;
}

void Mpu9250Init(const Mpu9250Config &cfg) {
  config = cfg;
  if (!mpu9250.Begin()) {
    MsgError("unable to establish communication with MPU-9250");
  }
  if (!mpu9250.ConfigAccelRange(cfg.accel_range_g)) {
    MsgError("unable to set MPU-9250 accel range");
  }
  if (!mpu9250.ConfigGyroRange(cfg.gyro_range_dps)) {
    MsgError("unable to set MPU-9250 gyro range");
  }
  if (!mpu9250.ConfigDlpfBandwidth(cfg.dlpf_hz)) {
    MsgError("unable to set MPU-9250 DLPF bandwidth");
  }
  time_ms = 0;
  while (time_ms < INIT_TIME_MS) {
    if (mpu9250.Read()) {
      gyro_radps = config.rotation * mpu9250.gyro_radps();
      gx.Update(gyro_radps[0]);
      gy.Update(gyro_radps[1]);
      gz.Update(gyro_radps[2]);
    }
  }
  gyro_bias_radps[0] = -gx.mean();
  gyro_bias_radps[1] = -gy.mean();
  gyro_bias_radps[2] = -gz.mean();
}

void Mpu9250EnableDrdy() {
  if (!mpu9250.EnableDrdyInt()) {
    MsgError("unable to enable MPU-9250 DRDY interrupt");
  }
  if (!mpu9250.ConfigSrd(MPU9250_SRD)) {
    MsgError("unable to set MPU-9250 SRD");
  }
}

void Mpu9250Read() {
  status = mpu9250.Read();
  if (status) {
    time_ms = 0;
  } else {
    MsgWarning("error reading MPU-9250");
  }
  healthy = (time_ms < HEALTHY_TIMEOUT_MS);
}

void Mpu9250ImuData(ImuData * const data) {
  data->installed = true;
  data->new_mag_data = false;
  data->new_imu_data = status;
  data->healthy = healthy;
  if (data->new_imu_data) {
    data->new_mag_data = mpu9250.new_mag_data();
    data->die_temp_c = mpu9250.die_temp_c();
    accel_mps2 = config.accel_scale * config.rotation * mpu9250.accel_mps2() +
                 config.accel_bias_mps2;
    gyro_radps = config.rotation * mpu9250.gyro_radps() + gyro_bias_radps;
    data->accel_mps2[0] = accel_mps2[0];
    data->accel_mps2[1] = accel_mps2[1];
    data->accel_mps2[2] = accel_mps2[2];
    data->gyro_radps[0] = gyro_radps[0];
    data->gyro_radps[1] = gyro_radps[1];
    data->gyro_radps[2] = gyro_radps[2];
    if (data->new_mag_data)
    mag_ut = config.mag_scale * config.rotation * mpu9250.mag_ut() +
             config.mag_bias_ut;
    data->mag_ut[0] = mag_ut[0];
    data->mag_ut[1] = mag_ut[1];
    data->mag_ut[2] = mag_ut[2];
  }
}
