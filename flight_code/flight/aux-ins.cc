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

#include "global_defs.h"
#include "flight/aux-ins.h"
#include "flight/msg.h"
#include "navigation.h"
#include "eigen.h"

namespace {
AuxInsConfig cfg_;
InsData *ins_;
bool initialized_ = false;
bool home_pos_ = false;
Eigen::Vector3d llh_, llh0_, ned_;
}  // namespace

void AuxInsInit(const AuxInsConfig &ref) {
  cfg_ = ref;
}

void AuxInsRun(AircraftData & ref, AuxInsData * const data) {
  if (!initialized_) {
    switch (cfg_.ins_source) {
      case AUX_INS_BFS: {
        ins_ = &ref.bfs_ins;
        break;
      }
      #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
          defined(__FMU_R_V2_BETA__)
      case AUX_INS_VECTOR_NAV: {
        if (ref.sensor.vector_nav_gnss.installed) {
          ins_ = &ref.vector_nav_ins;
        } else {
          MsgError("Aux INS source set to VectorNav, which is not installed");
        }
        break;
      }
      #endif
    }
    initialized_ = true;
  } else {
    if (ins_->initialized) {
      if (!home_pos_) {
        llh0_[0] = ins_->lat_rad;
        llh0_[1] = ins_->lon_rad;
        llh0_[2] = ins_->alt_wgs84_m;
        home_pos_ = true;
      }
      data->gnd_spd_mps = std::sqrt(ins_->ned_vel_mps[0] *
                                    ins_->ned_vel_mps[0] +
                                    ins_->ned_vel_mps[1] *
                                    ins_->ned_vel_mps[1]);
      data->gnd_track_rad = std::atan2(ins_->ned_vel_mps[1],
                                       ins_->ned_vel_mps[0]);
      data->flight_path_rad = std::atan2(-1.0f * ins_->ned_vel_mps[2],
                                         data->gnd_spd_mps);
      data->home_alt_wgs84_m = llh0_[2];
      data->home_lat_rad = llh0_[0];
      data->home_lon_rad = llh0_[1];
      llh_[0] = ins_->lat_rad;
      llh_[1] = ins_->lon_rad;
      llh_[2] = ins_->alt_wgs84_m;
      ned_ = bfs::lla2ned(llh_, llh0_, bfs::AngPosUnit::RAD);
      data->ned_pos_m[0] = ned_[0];
      data->ned_pos_m[1] = ned_[1];
      data->ned_pos_m[2] = ned_[2];
    }
  }
}