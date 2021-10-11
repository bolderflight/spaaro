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
#include "flight/sys.h"
#include "flight/sensors.h"
#include "flight/effectors.h"
#include "flight/nav.h"
#include "flight/control.h"
#include "flight/datalog.h"
#include "flight/telem.h"
#if defined(__FMU_R_V1__)
#include "flight/gpio.h"
#endif

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
  SensorsRead(&data.sensor);
  /* GPIO data */
  #if defined(__FMU_R_V1__)
  GpioRead(&data.gpio);
  #endif
  /* Nav filter */
  NavRun(data.sensor, &data.nav);
  /* Control laws */
  ControlRun(data.sys, data.sensor, data.nav, data.telem, &data.control);
  /* Command effectors */
  if (data.sensor.inceptor.failsafe) {
    EffectorsCmd(false, false, data.control);
  } else {
    EffectorsCmd(data.sensor.inceptor.throttle_en, true, data.control);
  }
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
  /* Init GPIO */
  #if defined(__FMU_R_V1__)
  GpioInit(config.gpio);
  #endif
  /* Init nav */
  NavInit(config.nav);
  /* Init effectors */
  EffectorsInit(config.effector);
  /* Init control */
  ControlInit();
  /* Init telemetry */
  TelemInit(config, &data.telem);
  /* Init datalog */
  DatalogInit();
  /* Attach data ready interrupt */
  attachInterrupt(IMU_DRDY, run, RISING);
  while (1) {
    /* Flush datalog */
    DatalogFlush();
  }
}
