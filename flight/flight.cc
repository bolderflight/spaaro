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

/* Aircraft sensors */
Sensors sensors;
/* Aircraft effectors */
Effectors effectors;
/* Aircraft data */
AircraftData data;
/* Timer for sending effector commands */
IntervalTimer effector_timer;

/* ISR to send effector commands */
void send_effectors() {
  /* Stop the effector timer */
  effector_timer.end();
  /* Send effector commands */
  EffectorsWrite(&effectors);
}

/* ISR to gather sensor data and run VMS */
void run() {
  /* Start the effector timer */
  effector_timer.begin(send_effectors, EFFECTOR_DELAY_US);
  /* System data */
  SysRead(&data.sys);
  /* Sensor data */
  SensorsRead(&data.sensor, &sensors);
  /* Nav filter */
  NavRun(data.sensor, &data.nav);
  /* Control laws */
  ControlRun(data, &data.control);
  /* Command effectors */
  EffectorsCmd(data.sensor.inceptor.throttle_en, data.control, &effectors);
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
  SensorsInit(config.sensor, &sensors);
  /* Init nav */
  NavInit(config.nav);
  /* Init effectors */
  EffectorsInit(config.effector, &effectors);
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
