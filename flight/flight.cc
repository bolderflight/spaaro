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
#include "flight/sys_mon.h"
#include "flight/inceptor.h"
#include "flight/imu.h"
#include "flight/gnss.h"
#include "flight/airdata.h"
#include "flight/sensor.h"
#include "flight/nav.h"
#include "flight/control.h"
#include "flight/effector.h"
#include "flight/datalog.h"
#include "flight/telemetry.h"

/* Whether to boot in DEBUG mode */
bool DEBUG = true;
/* Aircraft data */
AircraftData data;
/* Timer for sending effector commands */
IntervalTimer effector_timer;
/* ISR to send effector commands */
void send_effector() {
  /* Stop the effector timer */
  effector_timer.end();
  /* Send effector commands */
  EffectorWrite();
}
/* ISR to collect data and run VMS */
void run() {
  /* Start the effector timer */
  effector_timer.begin(send_effector, EFFECTOR_DELAY_US);
  /* Read system data */
  SysMonRead(&data.sys_mon);
  /* Read inceptors */
  InceptorRead(&data.inceptor);
  /* Read sensors */
  SensorRead(&data.sensor);
  /* Run guidance */
  // GuidanceRun();
  /* Run nav filter */
  NavRun(data.sensor, &data.nav);
  /* Run control */
  ControlRun(data.sys_mon, data.inceptor, data.sensor, data.nav,
             &data.control, &data.effector);
  /* Add to datalog */
  DatalogAdd(data);
  /* Telemetry Send and Receive */
  TelemTxRx(data);
  /* Frame duration */
  SysMonFrameEnd();
}

int main() {
  /* Init the message bus */
  MsgBegin();
  /* Init system monitoring */
  SysMonInit();
  /* Init inceptors */
  InceptorInit();
  /* Init sensors */
  SensorInit();
  /* Init guidance */
  // GuidanceInit();
  /* Init nav filter */
  NavInit();
  /* Init control */
  ControlInit();
  /* Init effectors */
  EffectorInit();
  /* Init datalog */
  DatalogInit();
  /* Init telemetry */
  TelemInit();
  /* Register Data ISR */
  RegisterDataIsr(run);
  while(1) {
    /* Flush data to SD */
    DatalogFlush();
  }
}
