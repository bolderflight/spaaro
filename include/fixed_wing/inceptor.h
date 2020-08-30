/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FIXED_WING_INCEPTOR_H_
#define INCLUDE_FIXED_WING_INCEPTOR_H_

#include "fixed_wing/global_defs.h" 

namespace inceptor {

void Init();
void Read(InceptorData *ptr);

}  // namespace inceptor

#endif  // INCLUDE_FIXED_WING_INCEPTOR_H_
