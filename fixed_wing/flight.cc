/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "core/core.h"
#include "fixed_wing/print_msg.h"
#include "fixed_wing/hardware_defs.h"
#include "fixed_wing/global_defs.h"
#include "fixed_wing/inceptor.h"
#include "fixed_wing/ins.h"
#include "fixed_wing/airdata.h"
#include "fixed_wing/control.h"
#include "fixed_wing/effector.h"

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
  /* Controls */
  controls::Run(data, &data.control);
  /* Effectors */
  effector::Cmd(data.control.cmds);
}

int main() {
  /* Init debug messages */
  print::Begin();
  print::DeviceInfo();
  /* Init inceptors */
  inceptor::Init();
  /* Init sensors and sensor processing */
  ins::Init();
  airdata::Init();
  /* Init controls */
  controls::Init();
  /* Init effectors */
  effector::Init();
  /* Attach data acquisition interrupt to INS data ready */
  ins::AttachCallback(IMU_DRDY, daq);
  while(1) {

  }
}
