/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "fixed_wing/datalog.h"
#include "fixed_wing/print_msg.h"
#include "fixed_wing/hardware_defs.h"
#include "fixed_wing/global_defs.h"
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
Logger<450> logger_(&sd_);
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
  datalog_msg_.imu_accel_x_mps2 = ref.ins.imu.accel_mps2(0);
  datalog_msg_.imu_accel_y_mps2 = ref.ins.imu.accel_mps2(1);
  datalog_msg_.imu_accel_z_mps2 = ref.ins.imu.accel_mps2(2);
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
