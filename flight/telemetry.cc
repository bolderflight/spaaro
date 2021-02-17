/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "flight/telemetry.h"
#include "flight/print_msg.h"
#include "flight/hardware_defs.h"
#include "flight/global_defs.h"
// #include "mavlink/mavlink.h"
// #include "global_defs/global_defs.h"
#include <cmath>

namespace telemetry {
namespace {
// /* MavLink*/
// MavLink telem_(&TELEM_UART, TELEM_ID, MavLink::VehicleType::FIXED_WING);
// /* Telemetry SRD */
// static constexpr int TELEM_SRD_ = FRAME_RATE_HZ / TELEM_RATE_HZ - 1;
// int telem_counter_ = 0;
// /* Ground speed and heading */
// float ground_speed_mps_, heading_rad_;
}  // anonymous

void Init() {
  // print::Info("Initializing telemetry...");
  // telem_.Begin(TELEM_BAUD);
  // print::Info("done.\n");
}
void Send(const AircraftData &ref) {
  // if (telem_counter_ >= TELEM_SRD_) {
  //   /* Send attitude data */
  //   telem_.SendAttitude(static_cast<uint32_t>(ref.time_s * 1000), 
  //     ref.ins.ekf.roll_rad,
  //     ref.ins.ekf.pitch_rad,
  //     ref.ins.ekf.yaw_rad,
  //     ref.ins.ekf.gyro_radps(0),
  //     ref.ins.ekf.gyro_radps(1),
  //     ref.ins.ekf.gyro_radps(2)
  //   );
  //   /* Send HUD data */
  //   /* Ground speed = sqrt(vn^2 + ve^2) */
  //   ground_speed_mps_ = sqrtf(ref.ins.ekf.ned_vel_mps(0) * ref.ins.ekf.ned_vel_mps(0) + 
  //     ref.ins.ekf.ned_vel_mps(1) * ref.ins.ekf.ned_vel_mps(1));
  //   /* Heading = atan2(ve, vn) */
  //   heading_rad_ = atan2f(ref.ins.ekf.ned_vel_mps(1), ref.ins.ekf.ned_vel_mps(0));
  //   /* Limit to 0 - 360 */
  //   heading_rad_ = fmod(heading_rad_, 2.0f * global::constants::PI<float>);
  //   if (heading_rad_ < 0) {
  //     heading_rad_ += 2.0f * global::constants::PI<float>;
  //   }
  //   telem_.SendHud(static_cast<uint32_t>(ref.time_s * 1000),
  //     #ifdef HAVE_PITOT_STATIC
  //     ref.airdata.ias_mps,
  //     #else
  //     ground_speed_mps_,
  //     #endif
  //     ground_speed_mps_,
  //     ref.ins.ekf.lla_rad_m(2),
  //     -1.0f * ref.ins.ekf.ned_vel_mps(2), // EKF is positive down, this is positive up
  //     heading_rad_,
  //     0
  //   );
  //   /* Send battery data */
  //   telem_.SendBattery(ref.status.input_voltage);
  //   telem_counter_ = 0;
  // } else {
  //   telem_counter_++;
  // }
  // /* Send GNSS data */
  // if (ref.ins.gnss.updated) {
  //   telem_.SendGnss(static_cast<uint32_t>(ref.time_s * 1000),
  //     ref.ins.gnss.fix,
  //     ref.ins.gnss.num_satellites,
  //     ref.ins.gnss.lla_msl_rad_m(0),
  //     ref.ins.gnss.lla_msl_rad_m(1),
  //     ref.ins.gnss.lla_msl_rad_m(2),
  //     ref.ins.gnss.alt_wgs84_m,
  //     ref.ins.gnss.ground_speed_mps,
  //     ref.ins.gnss.ground_track_rad,
  //     ref.ins.gnss.horiz_accuracy_m,
  //     ref.ins.gnss.vert_accuracy_m,
  //     ref.ins.gnss.vel_accuracy_mps,
  //     ref.ins.gnss.track_accuracy_rad
  //   );
  // }
}
bool Update() {
  // return telem_.Update();
}

}  // namespace telemetry
