/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FIXED_WING_EFFECTOR_H_
#define INCLUDE_FIXED_WING_EFFECTOR_H_

#include "fixed_wing/global_defs.h" 

namespace effector {

void Init();
void Cmd(const EffectorCmds &ref);
void Write();

}  // namespace effector

#endif  // INCLUDE_FIXED_WING_EFFECTOR_H_
