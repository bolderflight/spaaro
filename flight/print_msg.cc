/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2019 Bolder Flight Systems
*/

#include "core/core.h"
#include "flight/print_msg.h"
#include "flight/hardware_defs.h"

namespace print {

namespace {
static constexpr bool DEBUG = true;
} // anonymous

void Begin()
{
  MSG_BUS.begin(115200);
  if (DEBUG) {
    while (!MSG_BUS) {}
  }
}

void DeviceInfo()
{
  MSG_BUS.println ("---------Bolder Flight Systems---------");
  MSG_BUS.println ("Flight Software");
  MSG_BUS.print   ("Version: ");
  MSG_BUS.print(MAJOR_VERSION);
  MSG_BUS.print(".");
  MSG_BUS.print(MINOR_VERSION);
  MSG_BUS.print(".");
  MSG_BUS.println(FIX_VERSION);
  MSG_BUS.println ("---------------------------------------");
}

void Info(std::string str)
{
  MSG_BUS.print(str);
}

void Warning(std::string str)
{
  MSG_BUS.print("\nWARNING: ");
  MSG_BUS.print(str);
}

void Error(std::string str)
{
  MSG_BUS.print("\nERROR: ");
  MSG_BUS.print(str);
  while (1) {}
}

} // print
