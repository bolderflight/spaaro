/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FLIGHT_TELEMETRY_H_
#define INCLUDE_FLIGHT_TELEMETRY_H_

#include "flight/global_defs.h"

namespace telemetry {
void Init();
void Send(const AircraftData &ref);
bool Update();
}  // namespace telemetry

#endif  // INCLUDE_FLIGHT_TELEMETRY_H_
