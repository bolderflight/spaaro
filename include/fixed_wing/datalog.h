/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FIXED_WING_DATALOG_H_
#define INCLUDE_FIXED_WING_DATALOG_H_

#include "fixed_wing/global_defs.h" 

namespace datalog {

void Init();
void Write(const AircraftData &ref);
void Flush();
void Close();

}  // namespace datalog

#endif  // INCLUDE_FIXED_WING_DATALOG_H_
