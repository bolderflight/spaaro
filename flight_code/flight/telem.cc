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

#include "flight/telem.h"
#include "global_defs.h"
#include "hardware_defs.h"
#include "mavlink.h"
#include "checksum.h"
#include "flight/msg.h"

namespace {
/* MavLink object */
bfs::MavLink<NUM_TELEM_PARAMS, NUM_UTM_MSG> telem_;
/* Temp storage for holding MissionItems as they are uploaded */
bfs::MissionItem temp_[NUM_FLIGHT_PLAN_POINTS];
/* Frame period, us */
static constexpr int16_t FRAME_PERIOD_US = FRAME_PERIOD_MS * 1000;
/* Parameter */
int32_t param_idx_;
static constexpr uint8_t PARAM_STORE_HEADER[] = {'B', 'F', 'S'};
static constexpr std::size_t PARAM_STORE_SIZE = sizeof(PARAM_STORE_HEADER) +
                                                NUM_TELEM_PARAMS *
                                                sizeof(float) +
                                                sizeof(uint16_t);
uint8_t param_buf[PARAM_STORE_SIZE];
bfs::Fletcher16 param_checksum;
uint16_t chk_computed, chk_read;
uint8_t chk_buf[2];
/* Config */
TelemConfig cfg_;
/* Telem initialize flags */
bool telem_initialized_ = false;;
/* Data pointers */
ImuData *imu_;
MagData *mag_;
GnssData *gnss_;
PresData *static_pres_, *diff_pres_;
AdcData *adc_;
InsData *ins_;
/* Effector */
std::array<int16_t, 16> effector_;
int NUM_SBUS = std::min(static_cast<std::size_t>(NUM_SBUS_CH),
                        effector_.size() - NUM_PWM_PINS);
}  // namespace

void TelemInit(const TelemConfig &cfg, TelemData * const ptr) {
  /* Config */
  cfg_ = cfg;
  telem_.hardware_serial(&TELEM_UART);
  #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
      defined(__FMU_R_MINI_V1__)
  if (cfg.gnss_rtk == TELEM_GNSS_RTK_EXT_GNSS1) {
    telem_.gnss_serial(&GNSS1_UART);
  } else if (cfg.gnss_rtk == TELEM_GNSS_RTK_EXT_GNSS2) {
    telem_.gnss_serial(&GNSS2_UART);
  }
  #elif defined(__FMU_R_V1__)
  if (cfg.gnss_rtk == TELEM_GNSS_RTK_EXT_GNSS1) {
    telem_.gnss_serial(&GNSS_UART);
  }
  #endif
  telem_.aircraft_type(cfg.aircraft_type);
  telem_.mission(ptr->flight_plan, NUM_FLIGHT_PLAN_POINTS, temp_);
  telem_.fence(ptr->fence, NUM_FENCE_POINTS);
  telem_.rally(ptr->rally, NUM_RALLY_POINTS);
  /* Load the telemetry parameters from EEPROM */
  for (std::size_t i = 0; i < PARAM_STORE_SIZE; i++) {
    param_buf[i] = EEPROM.read(i);
  }
  /* Check whether the parameter store has been initialized */
  /* If it hasn't... */
  if ((param_buf[0] != PARAM_STORE_HEADER[0]) ||
      (param_buf[1] != PARAM_STORE_HEADER[1]) ||
      (param_buf[2] != PARAM_STORE_HEADER[2])) {
    MsgInfo("Parameter storage not initialized, initializing...");
    /* Set the header */
    param_buf[0] = PARAM_STORE_HEADER[0];
    param_buf[1] = PARAM_STORE_HEADER[1];
    param_buf[2] = PARAM_STORE_HEADER[2];
    /* Zero out the parameters */
    for (std::size_t i = 0; i < NUM_TELEM_PARAMS * sizeof(float); i++) {
      param_buf[i + 3] = 0;
    }
    /* Compute the checksum */
    chk_computed = param_checksum.Compute(param_buf,
                                          sizeof(PARAM_STORE_HEADER) +
                                          NUM_TELEM_PARAMS * sizeof(float));
    chk_buf[0] = static_cast<uint8_t>(chk_computed >> 8);
    chk_buf[1] = static_cast<uint8_t>(chk_computed);
    param_buf[PARAM_STORE_SIZE - 2] = chk_buf[0];
    param_buf[PARAM_STORE_SIZE - 1] = chk_buf[1];
    /* Write to EEPROM */
    for (std::size_t i = 0; i < PARAM_STORE_SIZE; i++) {
      EEPROM.write(i, param_buf[i]);
    }
    MsgInfo("done.\n");
  /* If it has been initialized */
  } else {
    /* Check the checksum */
    chk_computed = param_checksum.Compute(param_buf,
                                          sizeof(PARAM_STORE_HEADER) +
                                          NUM_TELEM_PARAMS * sizeof(float));
    chk_buf[0] = param_buf[PARAM_STORE_SIZE - 2];
    chk_buf[1] = param_buf[PARAM_STORE_SIZE - 1];
    chk_read = static_cast<uint16_t>(chk_buf[0]) << 8 |
               static_cast<uint16_t>(chk_buf[1]);
    if (chk_computed != chk_read) {
      /* Parameter store corrupted, reset and warn */
      MsgWarning("Parameter storage corrupted, resetting...");
      /* Set the header */
      param_buf[0] = PARAM_STORE_HEADER[0];
      param_buf[1] = PARAM_STORE_HEADER[1];
      param_buf[2] = PARAM_STORE_HEADER[2];
      /* Zero out the parameters */
      for (std::size_t i = 0; i < NUM_TELEM_PARAMS * sizeof(float); i++) {
        param_buf[i + 3] = 0;
      }
      /* Compute the checksum */
      chk_computed = param_checksum.Compute(param_buf,
                                            sizeof(PARAM_STORE_HEADER) +
                                            NUM_TELEM_PARAMS * sizeof(float));
      chk_buf[0] = static_cast<uint8_t>(chk_computed >> 8);
      chk_buf[1] = static_cast<uint8_t>(chk_computed);
      param_buf[PARAM_STORE_SIZE - 2] = chk_buf[0];
      param_buf[PARAM_STORE_SIZE - 1] = chk_buf[1];
      /* Write to EEPROM */
      for (std::size_t i = 0; i < PARAM_STORE_SIZE; i++) {
        EEPROM.write(i, param_buf[i]);
      }
      MsgInfo("done.\n");
    } else {
      /* Copy parameter data to global defs */
      memcpy(ptr->param.data(), param_buf + sizeof(PARAM_STORE_HEADER),
             NUM_TELEM_PARAMS * sizeof(float));
      /* Update the parameter values in MAV Link */
      telem_.params(ptr->param);
    }
  }
  /* Begin communication */
  telem_.Begin(cfg.baud);
  /* Data stream rates */
  telem_.raw_sens_stream_period_ms(cfg.raw_sens_stream_preiod_ms);
  telem_.ext_status_stream_period_ms(cfg.ext_status_stream_period_ms);
  telem_.rc_chan_stream_period_ms(cfg.rc_channel_stream_period_ms);
  telem_.pos_stream_period_ms(cfg.pos_stream_period_ms);
  telem_.extra1_stream_period_ms(cfg.extra1_stream_period_ms);
  telem_.extra2_stream_period_ms(cfg.extra2_stream_period_ms);
}
void TelemUpdate(AircraftData &data, TelemData * const ptr) {
  if (!telem_initialized_) {
    switch (cfg_.imu_source) {
      #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
          defined(__FMU_R_V2_BETA__)
      case TELEM_IMU_VECTOR_NAV: {
        if (data.sensor.vector_nav_imu.installed) {
          imu_ = &data.sensor.vector_nav_imu;
        } else {
          MsgError("Telem IMU source set to VectorNav, which is not installed");
        }
        break;
      }
      #endif
      case TELEM_IMU_FMU: {
        if (data.sensor.fmu_imu.installed) {
          imu_ = &data.sensor.fmu_imu;
        } else {
          MsgError("Telem IMU source set to FMU, which is not installed");
        }
        break;
      }
    }
    switch (cfg_.mag_source) {
      case TELEM_MAG_FMU: {
        if (data.sensor.fmu_mag.installed) {
          mag_ = &data.sensor.fmu_mag;
        } else {
          MsgError("Telem mag source set to FMU, which is not installed");
        }
        break;
      }
      case TELEM_MAG_EXT_MAG: {
        if (data.sensor.ext_mag.installed) {
          mag_ = &data.sensor.ext_mag;
        } else {
          MsgError("Telem mag source set to external, which is not installed");
        }
        break;
      }
    }
    switch (cfg_.gnss_source) {
      #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
          defined(__FMU_R_V2_BETA__)
      case TELEM_GNSS_VECTOR_NAV: {
        if (data.sensor.vector_nav_gnss.installed) {
          gnss_ = &data.sensor.vector_nav_gnss;
        } else {
          MsgError("Telem GNSS source set to VectorNav, which is not installed");
        }
        break;
      }
      #endif
      case TELEM_GNSS_EXT_GNSS1: {
        if (data.sensor.ext_gnss1.installed) {
          gnss_ = &data.sensor.ext_gnss1;
        } else {
          MsgError("Telem GNSS source set to external GNSS1, which is not installed");
        }
        break;
      }
      #if defined(__FMU_R_V2__) || defined(__FMU_R_V2_BETA__) || \
        defined(__FMU_R_MINI_V1__)
      case TELEM_GNSS_EXT_GNSS2: {
        if (data.sensor.ext_gnss2.installed) {
          gnss_ = &data.sensor.ext_gnss2;
        } else {
          MsgError("Telem GNSS source set to external GNSS2, which is not installed");
        }
        break;
      }
      #endif
    }
    switch (cfg_.static_pres_source) {
      case TELEM_STATIC_PRES_FMU: {
        if (data.sensor.fmu_static_pres.installed) {
          static_pres_ = &data.sensor.fmu_static_pres;
        } else {
          MsgError("Telem static pres source set to FMU, which is not installed");
        }
        break;
      }
      case TELEM_STATIC_PRES_EXT_PRES1: {
        if (data.sensor.ext_pres1.installed) {
          if (data.sensor.ext_pres1.is_static_pres) {
            static_pres_ = &data.sensor.ext_pres1;
          } else {
            MsgError("Telem static pres source set to external pres1, which is not a static pressure transducer");
          }
        } else {
          MsgError("Telem static pres source set to external pres1, which is not installed");
        }
        break;
      }
      case TELEM_STATIC_PRES_EXT_PRES2: {
        if (data.sensor.ext_pres2.installed) {
          if (data.sensor.ext_pres2.is_static_pres) {
            static_pres_ = &data.sensor.ext_pres2;
          } else {
            MsgError("Telem static pres source set to external pres2, which is not a static pressure transducer");
          }
        } else {
          MsgError("Telem static pres source set to external pres2, which is not installed");
        }
        break;
      }
      case TELEM_STATIC_PRES_EXT_PRES3: {
        if (data.sensor.ext_pres3.installed) {
          if (data.sensor.ext_pres3.is_static_pres) {
            static_pres_ = &data.sensor.ext_pres3;
          } else {
            MsgError("Telem static pres source set to external pres3, which is not a static pressure transducer");
          }
        } else {
          MsgError("Telem static pres source set to external pres3, which is not installed");
        }
        break;
      }
      case TELEM_STATIC_PRES_EXT_PRES4: {
        if (data.sensor.ext_pres4.installed) {
          if (data.sensor.ext_pres4.is_static_pres) {
            static_pres_ = &data.sensor.ext_pres4;
          } else {
            MsgError("Telem static pres source set to external pres4, which is not a static pressure transducer");
          }
        } else {
          MsgError("Telem static pres source set to external pres4, which is not installed");
        }
        break;
      }
    }
    switch (cfg_.diff_pres_source) {
      case TELEM_DIFF_PRES_NONE: {
        break;
      }
      case TELEM_DIFF_PRES_EXT_PRES1: {
        if (data.sensor.ext_pres1.installed) {
          if (!data.sensor.ext_pres1.is_static_pres) {
            diff_pres_ = &data.sensor.ext_pres1;
          } else {
            MsgError("Telem diff pres source set to external pres1, which is not a diff pressure transducer");
          }
        } else {
          MsgError("Telem diff pres source set to external pres1, which is not installed");
        }
        break;
      }
      case TELEM_DIFF_PRES_EXT_PRES2: {
        if (data.sensor.ext_pres2.installed) {
          if (!data.sensor.ext_pres2.is_static_pres) {
            diff_pres_ = &data.sensor.ext_pres2;
          } else {
            MsgError("Telem diff pres source set to external pres2, which is not a diff pressure transducer");
          }
        } else {
          MsgError("Telem diff pres source set to external pres2, which is not installed");
        }
        break;
      }
      case TELEM_DIFF_PRES_EXT_PRES3: {
        if (data.sensor.ext_pres3.installed) {
          if (!data.sensor.ext_pres3.is_static_pres) {
            diff_pres_ = &data.sensor.ext_pres3;
          } else {
            MsgError("Telem diff pres source set to external pres3, which is not a diff pressure transducer");
          }
        } else {
          MsgError("Telem diff pres source set to external pres3, which is not installed");
        }
        break;
      }
      case TELEM_DIFF_PRES_EXT_PRES4: {
        if (data.sensor.ext_pres4.installed) {
          if (!data.sensor.ext_pres4.is_static_pres) {
            diff_pres_ = &data.sensor.ext_pres4;
          } else {
            MsgError("Telem diff pres source set to external pres4, which is not a diff pressure transducer");
          }
        } else {
          MsgError("Telem diff pres source set to external pres4, which is not installed");
        }
        break;
      }
    }
    switch (cfg_.ins_source) {
      #if defined(__FMU_R_V1__) || defined(__FMU_R_V2__) || \
          defined(__FMU_R_V2_BETA__)
      case TELEM_INS_VECTOR_NAV: {
        if (data.sensor.vector_nav_imu.installed) {
          ins_ = &data.state_est.vector_nav_ins;
        } else {
          MsgError("Telem INS source set to VectorNav, which is not installed");
        }
        break;
      }
      #endif
      case TELEM_INS_BFS: {
        ins_ = &data.state_est.bfs_ins;
        break;
      }
    }
    adc_ = &data.state_est.adc;
    telem_initialized_ = true;
  }
  /* System data */
  telem_.sys_time_us(data.sys.sys_time_us);
  telem_.cpu_load(data.sys.frame_time_us, FRAME_PERIOD_US);
  telem_.throttle_enabled(data.vms.motors_enabled);
  telem_.aircraft_mode(data.vms.mode);
  if (data.vms.motors_enabled) {
    telem_.aircraft_state(bfs::ACTIVE);
  } else {
    telem_.aircraft_state(bfs::STANDBY);
  }
  /* Battery data */
  #if defined(__FMU_R_V2__) || defined(__FMU_R_MINI_V1__)
  telem_.battery_volt(data.sensor.power_module.voltage_v);
  telem_.battery_current_ma(data.sensor.power_module.current_ma);
  #endif
  #if defined(__FMU_R_V1__)
  telem_.battery_volt(data.sys.input_volt);
  #endif
  telem_.battery_remaining_prcnt(data.vms.power_remaining_prcnt);
  telem_.battery_remaining_time_s(data.vms.flight_time_remaining_s);
  /* IMU data */
  telem_.accel_installed(imu_->installed);
  telem_.accel_healthy(imu_->healthy);
  telem_.imu_accel_x_mps2(imu_->accel_mps2[0]);
  telem_.imu_accel_y_mps2(imu_->accel_mps2[1]);
  telem_.imu_accel_z_mps2(imu_->accel_mps2[2]);
  telem_.gyro_installed(imu_->installed);
  telem_.gyro_healthy(imu_->healthy);
  telem_.imu_gyro_x_radps(imu_->gyro_radps[0]);
  telem_.imu_gyro_y_radps(imu_->gyro_radps[1]);
  telem_.imu_gyro_z_radps(imu_->gyro_radps[2]);
  telem_.imu_die_temp_c(imu_->die_temp_c);
  telem_.mag_installed(mag_->installed);
  telem_.mag_healthy(mag_->healthy);
  telem_.imu_mag_x_ut(mag_->mag_ut[0]);
  telem_.imu_mag_y_ut(mag_->mag_ut[1]);
  telem_.imu_mag_z_ut(mag_->mag_ut[2]);
  /* GNSS data */
  telem_.gnss_installed(gnss_->installed);
  telem_.gnss_healthy(gnss_->healthy);
  telem_.gnss_fix(gnss_->fix);
  telem_.gnss_num_sats(gnss_->num_sats);
  telem_.gnss_lat_rad(gnss_->lat_rad);
  telem_.gnss_lon_rad(gnss_->lon_rad);
  // telem_.gnss_alt_msl_m(data.sensor.gnss.alt_msl_m);
  telem_.gnss_alt_wgs84_m(gnss_->alt_wgs84_m);
  // telem_.gnss_track_rad(data.sensor.gnss.track_rad);
  // telem_.gnss_spd_mps(data.sensor.gnss.spd_mps);
  telem_.gnss_horz_acc_m(gnss_->horz_acc_m);
  telem_.gnss_vert_acc_m(gnss_->vert_acc_m);
  telem_.gnss_vel_acc_mps(gnss_->vel_acc_mps);
  /* Airdata */
  telem_.static_pres_installed(static_pres_->installed);
  telem_.static_pres_healthy(static_pres_->healthy);
  telem_.static_pres_pa(static_pres_->pres_pa);
  telem_.static_pres_die_temp_c(static_pres_->die_temp_c);
  if (diff_pres_ != nullptr) {
    telem_.diff_pres_installed(diff_pres_->installed);
    telem_.diff_pres_healthy(diff_pres_->healthy);
    telem_.diff_pres_pa(diff_pres_->pres_pa);
    telem_.diff_pres_die_temp_c(diff_pres_->die_temp_c);
  } else {
    telem_.diff_pres_installed(false);
  }
  /* INS data */
  telem_.nav_lat_rad(ins_->lat_rad);
  telem_.nav_lon_rad(ins_->lon_rad);
  // telem_.nav_alt_msl_m(data.nav.alt_msl_m);
  // telem_.nav_alt_agl_m(data.nav.alt_rel_m);
  // telem_.nav_north_pos_m(data.nav.ned_pos_m[0]);
  // telem_.nav_east_pos_m(data.nav.ned_pos_m[1]);
  // telem_.nav_down_pos_m(data.nav.ned_pos_m[2]);
  telem_.nav_north_vel_mps(ins_->ned_vel_mps[0]);
  telem_.nav_east_vel_mps(ins_->ned_vel_mps[1]);
  telem_.nav_down_vel_mps(ins_->ned_vel_mps[2]);
  // telem_.nav_gnd_spd_mps(data.nav.gnd_spd_mps);
  telem_.nav_ias_mps(adc_->ias_mps);
  telem_.nav_pitch_rad(ins_->pitch_rad);
  telem_.nav_roll_rad(ins_->roll_rad);
  telem_.nav_hdg_rad(ins_->heading_rad);
  telem_.nav_gyro_x_radps(ins_->gyro_radps[0]);
  telem_.nav_gyro_y_radps(ins_->gyro_radps[1]);
  telem_.nav_gyro_z_radps(ins_->gyro_radps[2]);
  /* Effector */
  for (std::size_t i = 0; i < NUM_PWM_PINS; i++) {
    effector_[i] = data.vms.pwm[i];
  }
  for (std::size_t i = 0; i < NUM_SBUS; i++) {
    effector_[i + NUM_PWM_PINS] = data.vms.sbus[i];
  }
  telem_.effector(effector_);
  /* Inceptor */
  telem_.inceptor_installed(true);
  telem_.inceptor_healthy(!data.sensor.inceptor.failsafe);
  telem_.throttle_prcnt(data.vms.throttle_cmd_prcnt);
  telem_.inceptor(data.sensor.inceptor.ch);
  /* Mission */
  if (data.vms.advance_waypoint) {
    telem_.AdvanceMissionItem();
  }
  /* Update */
  telem_.Update();
  /* Params */
  param_idx_ = telem_.updated_param();
  if (param_idx_ >= 0) {
    /* Update the value in global defs */
    ptr->param[param_idx_] = telem_.param(param_idx_);
    /* Update the parameter buffer value */
    memcpy(param_buf + sizeof(PARAM_STORE_HEADER) + param_idx_ * sizeof(float),
           &ptr->param[param_idx_], sizeof(float));
    /* Compute a new checksum */
    chk_computed = param_checksum.Compute(param_buf,
                                          sizeof(PARAM_STORE_HEADER) +
                                          NUM_TELEM_PARAMS * sizeof(float));
    chk_buf[0] = static_cast<uint8_t>(chk_computed >> 8);
    chk_buf[1] = static_cast<uint8_t>(chk_computed);
    param_buf[PARAM_STORE_SIZE - 2] = chk_buf[0];
    param_buf[PARAM_STORE_SIZE - 1] = chk_buf[1];
    /* Write to EEPROM */
    for (std::size_t i = 0; i < sizeof(float); i++) {
      std::size_t addr = i + sizeof(PARAM_STORE_HEADER) +
                         param_idx_ * sizeof(float);
      EEPROM.write(addr, param_buf[addr]);
    }
    EEPROM.write(PARAM_STORE_SIZE - 2, param_buf[PARAM_STORE_SIZE - 2]);
    EEPROM.write(PARAM_STORE_SIZE - 1, param_buf[PARAM_STORE_SIZE - 1]);
  }
  /* Flight plan */
  ptr->waypoints_updated = telem_.mission_updated();
  ptr->current_waypoint = telem_.active_mission_item();
  ptr->num_waypoints = telem_.num_mission_items();
  /* Fence */
  ptr->fence_updated = telem_.fence_updated();
  ptr->num_fence_items = telem_.num_fence_items();
  /* Rally */
  ptr->rally_points_updated = telem_.rally_points_updated();
  ptr->num_rally_points = telem_.num_rally_points();
}
