/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#ifndef FLIGHT_CODE_INCLUDE_FLIGHT_DATALOG_H_
#define FLIGHT_CODE_INCLUDE_FLIGHT_DATALOG_H_

#include "global_defs.h"
#include "logger/logger.h"

/* Union for wp params*/
union wp_param_type{
  int i;
  float f;
  uint8_t bytes[4];
};

void DatalogInit();
void DatalogAdd(const AircraftData &ref);
void DatalogClose();
void DatalogFlush();
void WaypointWrite(const AircraftData &ref);
void WaypointRead (TelemData * const ptr);
void write_union(uint8_t byte[4], File32 &file_);
void read_union(wp_param_type * const ptr, File32 &file_);

#endif  // FLIGHT_CODE_INCLUDE_FLIGHT_DATALOG_H_
