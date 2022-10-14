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

#include "flight/msg.h"
#include "hardware_defs.h"
#include "flight/config.h"
#include "./version.h"

void MsgBegin() {
  MSG_BUS.begin(115200);
  if (DEBUG) {
    while (!MSG_BUS) {}
  }
  MSG_BUS.println("---------Bolder Flight Systems---------");
  MSG_BUS.println("Flight Software");
  MSG_BUS.print("Version: ");
  MSG_BUS.println(PROJECT_VERSION);
  MSG_BUS.println("---------------------------------------");
}

void MsgInfo(const char * str) {
  MSG_BUS.print(str);
}

void MsgWarning(const char * str) {
  MSG_BUS.print("\nWARNING: ");
  MSG_BUS.print(str);
}

void MsgError(const char * str) {
  MSG_BUS.print("\nERROR: ");
  MSG_BUS.print(str);
  while (1) {}
}
