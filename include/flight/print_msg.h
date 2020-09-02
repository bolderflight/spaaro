/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FLIGHT_PRINT_MSG_H_
#define INCLUDE_FLIGHT_PRINT_MSG_H_

#include <string>

namespace print {
/* Initializes the printer */
void Begin();
/* Prints the software version and serial number */
void DeviceInfo();
/* Prints info to the serial terminal */
void Info(std::string str);
/* Prints warning message */
void Warning(std::string str);
/* Prints error message */
void Error(std::string str);
} // print

#endif  // INCLUDE_FLIGHT_PRINT_MSG_H_
