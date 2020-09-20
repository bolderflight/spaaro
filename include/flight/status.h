/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FLIGHT_STATUS_H_
#define INCLUDE_FLIGHT_STATUS_H_

#include "flight/global_defs.h" 

namespace status {

void Init();
void Read(StatusData *ptr);

}  // namespace status

#endif  // INCLUDE_FLIGHT_STATUS_H_
