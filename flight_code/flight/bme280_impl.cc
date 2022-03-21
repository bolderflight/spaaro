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

#include "flight/bme280_impl.h"
#include "flight/global_defs.h"
#include "flight/hardware_defs.h"
#include "flight/msg.h"
#include "bme280.h"

namespace {
/* BME280 object */
Bme280 bme280(&SPI_BUS, BME280_CS);
/* Interim data */
bool status, healthy;
elapsedMillis time_ms;
}

void Bme280Init() {
  if (!bme280.Begin()) {
    MsgError("unable to establish communication with BME280");
  }
}
void Bme280Read() {
  status = bme280.Read();
  if (status) {
    time_ms = 0;
  } else {
    MsgWarning("error reading BME280");
  }
  healthy = (time_ms < HEALTHY_TIMEOUT_MS);
}

void Bme280PresData(PresData * const data) {
  data->installed = true;
  data->healthy = healthy;
  data->new_data = status;
  if (data->new_data) {
    data->die_temp_c = bme280.die_temp_c();
    data->pres_pa = bme280.pres_pa();
  }
}
