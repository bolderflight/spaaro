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
#include "flight/global_defs.h"
#include "flight/config.h"
#include "flight/sensors.h"
#include "flight/effectors.h"
#include "flight/sys.h"
#include "flight/vms.h"
#include "flight/datalog.h"
#include "flight/telem.h"
#include "flight/airdata_est.h"
#include "flight/vector_nav_impl.h"
#include "flight/bfs_ekf.h"

/* Aircraft data */
AircraftData data;
/* Timer for sending effector commands */
IntervalTimer effector_timer;
/* ISR to send effector commands */
void send_effectors() {
  /* Stop the effector timer */
  effector_timer.end();
  #if defined(__FMU_R_V1__)
  /* Pulse the BFS bus */
  digitalWriteFast(BFS_INT1, LOW);
  digitalWriteFast(BFS_INT2, HIGH);
  #endif
  /* Send effector commands */
  EffectorsWrite();
}
/* ISR to gather sensor data and run VMS */
void run() {
  /* Start the effector timer */
  effector_timer.begin(send_effectors, EFFECTOR_DELAY_US);
  #if defined(__FMU_R_V1__)
  /* Pulse the BFS bus */
  digitalWriteFast(BFS_INT1, HIGH);
  digitalWriteFast(BFS_INT2, LOW);
  #endif
  /* System data */
  SysRead(&data.sys);
  /* Sensor data */
  ReadSensors(&data.sensor);
  /* Nav filter */
  AirDataEst(data.sensor, &data.nav.airdata);
  BfsNavRun(data.sensor, &data.nav.bfs_ekf);
  VectorNavInsData(&data.nav.vector_nav);
  /* VMS */
  VmsRun(data.sys, data.sensor, data.nav, data.telem, &data.vms);
  /* Command effectors */
  EffectorsCmd(data.vms.sbus, data.vms.pwm);
  /* Datalog */
  DatalogAdd(data);
  /* Telemetry */
  TelemUpdate(data, &data.telem);
  /* Frame duration */
  SysFrameEnd();
}

int main() {
  /* Init the message bus */
  MsgBegin();
  /* Init system */
  SysInit();
  /* Init sensors */
  InitSensors(config.sensor);
  /* Init nav filter */
  AirDataInit(config.airdata);
  BfsNavInit(config.bfs_ekf);
  /* Init effectors */
  EffectorsInit();
  /* Init VMS */
  VmsInit();
  /* Init telemetry */
  TelemInit(config, &data.telem);
  /* Init datalog */
  DatalogInit();
  /* Attach data ready interrupt */
  if (config.sensor.drdy_source == DRDY_MPU9250) {
    attachInterrupt(MPU9250_DRDY, run, RISING);
  } else {
    attachInterrupt(VN_DRDY, run, RISING);
  }
  while (1) {
    /* Flush datalog */
    DatalogFlush();
  }
}
