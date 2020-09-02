/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "flight/ins.h"
#include "flight/print_msg.h"
#include "flight/hardware_defs.h"
#include "flight/global_defs.h"
#include "mpu9250/mpu9250.h"
#include "ublox/ublox.h"
#include "navigation/navigation.h"

namespace ins {
namespace {
/* MPU-9250 IMU */
sensors::Mpu9250 imu_(&IMU_SPI_BUS, IMU_CS);
static constexpr sensors::Mpu9250::AccelRange ACCEL_RANGE_ = sensors::Mpu9250::ACCEL_RANGE_16G;
static constexpr sensors::Mpu9250::GyroRange Gyro_RANGE_ = sensors::Mpu9250::GYRO_RANGE_2000DPS;
static constexpr sensors::Mpu9250::DlpfBandwidth DLPF_ = sensors::Mpu9250::DLPF_BANDWIDTH_20HZ;
static constexpr unsigned int SRD_ = 1000 / FRAME_RATE_HZ - 1;
static Eigen::Matrix3f ROTATION_ = (Eigen::Matrix3f() << 0, 1, 0, -1, 0, 0, 0, 0, 1).finished();
/* uBlox GNSS */
sensors::Ublox gnss_(&GNSS_UART);
/* EKF-15 state INS */
navigation::Ekf15State ekf_;
/* Minimum number of satellites */
static constexpr int MIN_SAT_ = 12;
};  // anonymous

void Init() {
  /* Init IMU */
  print::Info("Initializing IMU...");
  /* Initialize communication */
  if (!imu_.Begin()) {
    print::Error("Unable to initialize communication with IMU.");
  }
  /* Set the rotation */
  imu_.rotation(ROTATION_);
  /* Set the accel range */
  if (!imu_.accel_range(ACCEL_RANGE_)) {
    print::Error("Unable to set IMU accel full-scale range.");
  }
  /* Set the gryo range */
  if (!imu_.gyro_range(Gyro_RANGE_)) {
    print::Error("Unable to set IMU gyro full-scale range.");
  }
  /* Set the DLPF */
  if (!imu_.dlpf_bandwidth(DLPF_)) {
    print::Error("Unable to set IMU DLPF bandwidth.");
  }
  /* Set the SRD */
  if (!imu_.sample_rate_divider(SRD_)) {
    print::Error("Unable to set IMU sample rate divider.");
  }
  /* Enable data ready interrupt */
  if (!imu_.EnableDrdyInt()) {
    print::Error("Unable to set data ready callback.");
  }
  print::Info("done.\n");
  /* Init GNSS */
  print::Info("Initializing GNSS...");
  /* Initialize communication */
  if (!gnss_.Begin(GNSS_BAUD)) {
    print::Error("Unable to initialize communication with GNSS receiver.");
  }
  print::Info("done.\n");
  /* Initialize INS */
  print::Info("Waiting for GNSS lock...");
  while ((gnss_.num_satellites() < MIN_SAT_) && (gnss_.fix() != sensors::Ublox::FIX_3D)) {
    gnss_.Read();
  }
  print::Info("done.\n");
  print::Info("Initializing EKF...");
  /* Get updated IMU data */
  while (!imu_.Read()) {}
  /* Initialize the EKF states */
  ekf_.Initialize(imu_.accel_mps2(), imu_.gyro_radps(), imu_.mag_ut(), gnss_.ned_velocity_mps(), gnss_.lla_rad_m());
  print::Info("done.\n");
}
void AttachCallback(uint8_t int_pin, void (*function)()) {
  imu_.DrdyCallback(int_pin, function);
}
void Read(InsData *ptr) {
  if(!ptr) {return;}
  /* Read the IMU */
  if (imu_.Read()) {
    ptr->imu.accel_mps2 = imu_.accel_mps2();
    ptr->imu.gyro_radps = imu_.gyro_radps();
    ptr->imu.mag_ut = imu_.mag_ut();
    ptr->imu.die_temp_c = imu_.die_temperature_c();
    /* EKF time update */
    ekf_.TimeUpdate(imu_.accel_mps2(), imu_.gyro_radps(), FRAME_PERIOD_S);
    ptr->ekf.accel_bias_mps2 = ekf_.accel_bias_mps2();
    ptr->ekf.gyro_bias_radps = ekf_.gyro_bias_radps();
    ptr->ekf.accel_mps2 = ekf_.accel_mps2();
    ptr->ekf.gyro_radps = ekf_.gyro_radps();
    ptr->ekf.ned_vel_mps = ekf_.ned_vel_mps();
    ptr->ekf.lla_rad_m = ekf_.lla_rad_m();
    ptr->ekf.pitch_rad = ekf_.pitch_rad();
    ptr->ekf.roll_rad = ekf_.roll_rad();
    ptr->ekf.yaw_rad = ekf_.yaw_rad();
  } 
  /* Check if GNSS packet available */
  if (gnss_.Read()) {
    ptr->gnss.year = gnss_.year();
    ptr->gnss.month = gnss_.month();
    ptr->gnss.day = gnss_.day();
    ptr->gnss.hour = gnss_.hour();
    ptr->gnss.min = gnss_.min();
    /* 
    * Storing seconds as a float with millisecond precision. 
    * Using integer division on nano-sec to truncate to milliseconds
    * before casting to a float and converting milliseconds to seconds.
    */
    ptr->gnss.sec = static_cast<float>(gnss_.sec()) + static_cast<float>(gnss_.nano_sec() / 1000) / 1000000.0f;
    ptr->gnss.fix = static_cast<uint8_t>(gnss_.fix());
    ptr->gnss.num_satellites = gnss_.num_satellites();
    ptr->gnss.ned_vel_mps = gnss_.ned_velocity_mps();
    ptr->gnss.lla_rad_m = gnss_.lla_rad_m();
    ptr->gnss.time_accuracy_ns = gnss_.time_accuracy_ns();
    ptr->gnss.horiz_accuracy_m = gnss_.horizontal_accuracy_m();
    ptr->gnss.vert_accuracy_m = gnss_.vertical_accuracy_m();
    ptr->gnss.vel_accuracy_mps = gnss_.velocity_accuracy_mps();
    /* EKF measurement update */
    ekf_.MeasurementUpdate(gnss_.ned_velocity_mps(), gnss_.lla_rad_m());
    ptr->ekf.accel_bias_mps2 = ekf_.accel_bias_mps2();
    ptr->ekf.gyro_bias_radps = ekf_.gyro_bias_radps();
    ptr->ekf.accel_mps2 = ekf_.accel_mps2();
    ptr->ekf.gyro_radps = ekf_.gyro_radps();
    ptr->ekf.ned_vel_mps = ekf_.ned_vel_mps();
    ptr->ekf.lla_rad_m = ekf_.lla_rad_m();
    ptr->ekf.pitch_rad = ekf_.pitch_rad();
    ptr->ekf.roll_rad = ekf_.roll_rad();
    ptr->ekf.yaw_rad = ekf_.yaw_rad();
  }
}

}  // namespace ins
