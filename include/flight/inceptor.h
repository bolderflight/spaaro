/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FLIGHT_INCEPTOR_H_
#define INCLUDE_FLIGHT_INCEPTOR_H_

#include "flight/global_defs.h" 

namespace inceptor {

void Init();
void Read(InceptorData *ptr);

}  // namespace inceptor

#endif  // INCLUDE_FLIGHT_INCEPTOR_H_
