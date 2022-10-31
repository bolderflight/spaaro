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

#include "hardware_defs.h"
#include "global_defs.h"
#include "flight/config.h"
#include "flight/msg.h"
#include "flight/sys.h"
#include "flight/sensors.h"
#if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
    defined(__FMU_R_V2_BETA__)
#include "drivers/spaaro-vector-nav.h"
#endif
#include "flight/adc.h"
#include "flight/bfs-ins.h"
#include "flight/aux-ins.h"
#include "flight/effectors.h"
#include "flight/datalog.h"
#include "flight/telem.h"
#include "flight/vms.h"

/* Aircraft data */
AircraftData data;
/* Timer for sending effector commands */
IntervalTimer effector_timer;

/* ISR to send effector commands */
void send_effectors() {
  /* Stop the effector timer */
  effector_timer.end();
  /* Send effector commands */
  EffectorsWrite();
}

/* ISR to gather sensor data and run VMS */
void run() {
  /* Start the effector timer */
  effector_timer.begin(send_effectors, EFFECTOR_DELAY_US);
  /* System data */
  SysRead(&data.sys);
  /* Sensor data */
  SensorsRead(&data.sensor);
  /* VectorNav */
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  VectorNavRead(&data.sensor.vector_nav_imu, &data.sensor.vector_nav_mag,
                &data.sensor.vector_nav_static_pres,
                &data.sensor.vector_nav_gnss, &data.vector_nav_ins);
  #endif
  /* Air data */
  AdcRun(data.sensor, &data.adc);
  /* INS */
  BfsInsRun(data.sensor, &data.bfs_ins);
  /* Aux INS */
  AuxInsRun(data, &data.aux_ins);
  /* VMS */
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  VmsRun(data.sys, data.sensor, data.bfs_ins, data.vector_nav_ins, data.adc,
         data.telem, &data.vms);
  #else
  VmsRun(data.sys, data.sensor, data.bfs_ins, data.adc,
         data.telem, &data.vms);
  #endif
  /* Command effectors */
  EffectorsCmd(data.vms);
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
  SensorsInit(config.sensor);
  /* Calibrate sensors */
  SensorsCal();
  /* Init VectorNav */
  #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
      defined(__FMU_R_V2_BETA__)
  VectorNavInit(config.vector_nav);
  #endif
  /* Init ADC */
  AdcInit(config.adc);
  /* Init INS */
  BfsInsInit(config.bfs_ins);
  /* Init Aux INS */
  AuxInsInit(config.aux_ins);
  /* Init effectors */
  EffectorsInit();
  /* Init VMS */
  VmsInit();
  /* Init telemetry */
  TelemInit(config.telem, &data.telem);
  /* Init datalog */
  DatalogInit();
  /* Attach data ready interrupt */
  attachInterrupt(IMU_DRDY, run, RISING);\
  while (1) {
    /* Flush datalog */
    DatalogFlush();
  }
}
