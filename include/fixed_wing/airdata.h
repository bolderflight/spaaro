/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FIXED_WING_AIRDATA_H_
#define INCLUDE_FIXED_WING_AIRDATA_H_

#include "fixed_wing/global_defs.h" 

namespace airdata {

void Init();
void Read(Airdata *ptr);

}  // namespace airdata

#endif  // INCLUDE_FIXED_WING_AIRDATA_H_
