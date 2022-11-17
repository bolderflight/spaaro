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

#include "flight/vms.h"
#ifdef __AUTOCODE__
  #include "./autocode.h"
#else
  #include "filter.h"
  #include "control.h"
  #include "excitation.h"
  #include "navigation.h"
  #include "airdata.h"
#endif

namespace {
#ifdef __AUTOCODE__
/* Autocode instance */
bfs::Autocode autocode;
#endif
}  // namespace

void VmsInit() {
#ifdef __AUTOCODE__
  autocode.initialize();
#endif
}
void VmsRun(const SysData &sys, const SensorData &sensor,
            const StateEstData &state_est, const TelemData &telem,
            VmsData *vms) {
  if (!vms) {return;}
#ifdef __AUTOCODE__
  autocode.Run(sys, sensor, state_est, telem, vms);
#endif
}
