/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FIXED_WING_INS_H_
#define INCLUDE_FIXED_WING_INS_H_

#include "fixed_wing/global_defs.h"

namespace ins {

  void Init();
  void AttachCallback(uint8_t int_pin, void (*function)());
  void Read(InsData *ptr);


}  // namespace ins

#endif  // INCLUDE_FIXED_WING_INS_H_
