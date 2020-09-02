/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FLIGHT_DATALOG_H_
#define INCLUDE_FLIGHT_DATALOG_H_

#include "flight/global_defs.h" 

namespace datalog {

void Init();
void Write(const AircraftData &ref);
void Flush();
void Close();

}  // namespace datalog

#endif  // INCLUDE_FLIGHT_DATALOG_H_
