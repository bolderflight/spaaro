/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FLIGHT_AIRDATA_H_
#define INCLUDE_FLIGHT_AIRDATA_H_

#include "flight/global_defs.h" 

namespace airdata {

void Init();
void Read(Airdata *ptr);

}  // namespace airdata

#endif  // INCLUDE_FLIGHT_AIRDATA_H_
