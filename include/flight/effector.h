/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FLIGHT_EFFECTOR_H_
#define INCLUDE_FLIGHT_EFFECTOR_H_

#include "flight/global_defs.h" 

namespace effector {

void Init();
void Cmd(const EffectorCmds &ref);
void Write();

}  // namespace effector

#endif  // INCLUDE_FLIGHT_EFFECTOR_H_
