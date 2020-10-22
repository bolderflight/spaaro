/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "core/core.h"
#include "flight/print_msg.h"
#include "flight/hardware_defs.h"
#include "flight/global_defs.h"
#include "flight/inceptor.h"
#include "flight/ins.h"
#include "flight/airdata.h"
#include "flight/control.h"
#include "flight/effector.h"
#include "flight/status.h"
#include "flight/datalog.h"
#include "flight/telemetry.h"

/* Timer for sending effector commands */
IntervalTimer effector_timer;

/* Aircraft data */
AircraftData data;

void send_effector() {
  /* Stop the effector timer */
  effector_timer.end();
  /* Send effector commands to actuators */
  effector::Write();
}

void daq() {
  /* Start the effector timer */
  effector_timer.begin(send_effector, EFFECTOR_DELAY_US);
  /* System time, us precision */
  data.time_s = static_cast<double>(micros64()) / 1e6;
  /* Read inceptors */
  inceptor::Read(&data.inceptor);
  /* Read sensors */
  ins::Read(&data.ins);
  airdata::Read(&data.airdata);
  status::Read(&data.status);
  /* Controls */
  controls::Run(data, &data.control);
  /* Effectors */
  effector::Cmd(data.control.cmds);
  /* Datalog */
  datalog::Write(data);
  /* Telemetry */
  telemetry::Send(data);
}

int main() {
  /* Init debug messages */
  print::Begin();
  print::DeviceInfo();
  /* Init inceptors */
  inceptor::Init();
  /* Init sensors and sensor processing */
  airdata::Init();
  ins::Init();
  status::Init();
  /* Init controls */
  controls::Init();
  /* Init effectors */
  effector::Init();
  /* Init datalog */
  datalog::Init();
  /* Init telemtry */
  telemetry::Init();
  /* Attach data acquisition interrupt to INS data ready */
  ins::AttachCallback(IMU_DRDY, daq);
  while(1) {
    /* Send heartbeat and check for messages */
    telemetry::Update();
    /* Flush datalog to SD */
    datalog::Flush();
  }
}
