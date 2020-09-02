/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FLIGHT_CONTROL_H_
#define INCLUDE_FLIGHT_CONTROL_H_

#include "flight/global_defs.h" 

namespace controls {

void Init();
void Run(const AircraftData &ref, ControlsData *ptr);

}  // namespace controls

#endif  // INCLUDE_FLIGHT_CONTROL_H_
