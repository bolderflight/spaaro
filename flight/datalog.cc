/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "flight/datalog.h"
#include "flight/print_msg.h"
#include "flight/hardware_defs.h"
#include "flight/global_defs.h"
#include "logger/logger.h"
#include "framing/framing.h"
#include "datalog.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

namespace datalog {
namespace {
/* Datalog file name */
std::string DATA_LOG_NAME_ = "flight_data";
/* Datalog message from protobuf */
DatalogMessage datalog_msg_;
/* SD card */
SdFatSdioEX sd_;
/* Logger object */
Logger<400> logger_(&sd_);
/* Framing */
framing::Encoder<DatalogMessage_size> encoder;
/* nanopb buffer for encoding */
uint8_t data_buffer_[DatalogMessage_size];
pb_ostream_t stream_;
}  // anonymous

void Init() {
  print::Info("Initializing datalog...");  
  /* Initialize SD card */
  sd_.begin();
  /* Initialize logger */
  int file_num = logger_.Init(DATA_LOG_NAME_);
  if (file_num < 0) {
    print::Error("Unable to initialize datalog.");
  }
  print::Info("done. Created datalog file ");
  print::Info(DATA_LOG_NAME_);
  print::Info(std::to_string(file_num));
  print::Info(".bfs\n");
  return;
}
void Write(const AircraftData &ref) {
  /* Fill the datalog message */
  datalog_msg_.time_s = ref.time_s;
  datalog_msg_.imu_accel_mps2[0] = ref.ins.imu.accel_mps2(0);
  datalog_msg_.imu_accel_mps2[1] = ref.ins.imu.accel_mps2(1);
  datalog_msg_.imu_accel_mps2[2] = ref.ins.imu.accel_mps2(2);
  datalog_msg_.imu_gyro_radps[0] = ref.ins.imu.gyro_radps(0);
  datalog_msg_.imu_gyro_radps[1] = ref.ins.imu.gyro_radps(1);
  datalog_msg_.imu_gyro_radps[2] = ref.ins.imu.gyro_radps(2);
  datalog_msg_.imu_mag_ut[0] = ref.ins.imu.mag_ut(0);
  datalog_msg_.imu_mag_ut[1] = ref.ins.imu.mag_ut(1);
  datalog_msg_.imu_mag_ut[2] = ref.ins.imu.mag_ut(2);
  datalog_msg_.gnss_updated = ref.ins.gnss.updated;
  datalog_msg_.utc_year = ref.ins.gnss.year;
  datalog_msg_.utc_month = ref.ins.gnss.month;
  datalog_msg_.utc_day = ref.ins.gnss.day;
  datalog_msg_.utc_hour = ref.ins.gnss.hour;
  datalog_msg_.utc_min = ref.ins.gnss.min;
  datalog_msg_.utc_sec = ref.ins.gnss.sec;
  datalog_msg_.gnss_fix = ref.ins.gnss.fix;
  datalog_msg_.gnss_num_sv = ref.ins.gnss.num_satellites;
  datalog_msg_.gnss_ned_vel_mps[0] = ref.ins.gnss.ned_vel_mps(0);
  datalog_msg_.gnss_ned_vel_mps[1] = ref.ins.gnss.ned_vel_mps(1);
  datalog_msg_.gnss_ned_vel_mps[2] = ref.ins.gnss.ned_vel_mps(2);
  datalog_msg_.gnss_lat_rad = ref.ins.gnss.lla_msl_rad_m(0);
  datalog_msg_.gnss_lon_rad = ref.ins.gnss.lla_msl_rad_m(1);
  datalog_msg_.gnss_alt_m = static_cast<float>(ref.ins.gnss.lla_msl_rad_m(2));
  datalog_msg_.ins_accel_mps2[0] = ref.ins.ekf.accel_mps2(0);
  datalog_msg_.ins_accel_mps2[1] = ref.ins.ekf.accel_mps2(1);
  datalog_msg_.ins_accel_mps2[2] = ref.ins.ekf.accel_mps2(2);
  datalog_msg_.ins_gyro_radps[0] = ref.ins.ekf.gyro_radps(0);
  datalog_msg_.ins_gyro_radps[1] = ref.ins.ekf.gyro_radps(1);
  datalog_msg_.ins_gyro_radps[2] = ref.ins.ekf.gyro_radps(2);
  datalog_msg_.ins_ned_vel_mps[0] = ref.ins.ekf.ned_vel_mps(0);
  datalog_msg_.ins_ned_vel_mps[1] = ref.ins.ekf.ned_vel_mps(1);
  datalog_msg_.ins_ned_vel_mps[2] = ref.ins.ekf.ned_vel_mps(2);
  datalog_msg_.ins_lat_rad = ref.ins.ekf.lla_rad_m(0);
  datalog_msg_.ins_lon_rad = ref.ins.ekf.lla_rad_m(1);
  datalog_msg_.ins_alt_m = static_cast<float>(ref.ins.ekf.lla_rad_m(2));
  datalog_msg_.pitch_rad = ref.ins.ekf.pitch_rad;
  datalog_msg_.roll_rad = ref.ins.ekf.roll_rad;
  datalog_msg_.yaw_rad = ref.ins.ekf.yaw_rad;
  #ifdef HAVE_PITOT_STATIC
  datalog_msg_.fmu_static_press_pa = ref.airdata.fmu_static_press.press_pa;
  datalog_msg_.diff_press_pa = ref.airdata.diff_press.press_pa;
  datalog_msg_.filt_diff_press_pa = ref.airdata.filt_diff_press_pa;
  datalog_msg_.ias_mps = ref.airdata.ias_mps;
  datalog_msg_.eas_mps = ref.airdata.eas_mps;
  #endif
  datalog_msg_.static_press_pa = ref.airdata.static_press.press_pa;
  datalog_msg_.filt_static_press_pa = ref.airdata.filt_static_press_pa;
  datalog_msg_.press_alt_m = ref.airdata.press_alt_m;
  datalog_msg_.agl_alt_m = ref.airdata.agl_alt_m;
  datalog_msg_.input_voltage = ref.status.input_voltage;
  datalog_msg_.regulated_voltage = ref.status.regulated_voltage;
  datalog_msg_.pwm_voltage = ref.status.pwm_voltage;
  datalog_msg_.sbus_voltage = ref.status.sbus_voltage;

  /* Encode */
  stream_ = pb_ostream_from_buffer(data_buffer_, sizeof(data_buffer_));
  if (!pb_encode(&stream_, DatalogMessage_fields, &datalog_msg_)) {
    print::Warning("Error encoding datalog.");
    return;
  }
  std::size_t msg_len = stream_.bytes_written;
  /* Framing */
  std::size_t bytes_written = encoder.Write(data_buffer_, msg_len);
  if (msg_len != bytes_written) {
    print::Warning("Error framing datalog.");
    return;
  }
  /* Write the data */
  logger_.Write(encoder.Data(), encoder.Size());
}
void Flush() {
  logger_.Flush();
}
void Close() {
  logger_.Close();
}

}  // namespace datalog
