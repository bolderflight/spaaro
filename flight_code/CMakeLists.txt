cmake_minimum_required(VERSION 3.14)
include(FetchContent)
FetchContent_Declare(
  mcu_support
  GIT_REPOSITORY https://github.com/bolderflight/mcu-support.git
  GIT_TAG v1.0.0
)
FetchContent_MakeAvailable(mcu_support)
# Setting up the toolchain
set(CMAKE_TOOLCHAIN_FILE "${mcu_support_SOURCE_DIR}/cmake/cortex.cmake")
# Project information
project(Flight
  VERSION 4.0.0
  DESCRIPTION "Flight software skeleton"
  LANGUAGES C CXX
)
# Export project info
configure_file(${CMAKE_SOURCE_DIR}/cmake/version.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/version.h)
# FMU version
if (DEFINED FMU)
  string(TOUPPER ${FMU} FMU)
endif()
if (FMU STREQUAL "V2")
  # FMU-R-V2
  set(MCU IMXRT1062_T41)
  add_definitions(
    -D__FMU_R_V2__
  )
elseif(FMU STREQUAL "V2-BETA")
  # FMU-R-V2-BETA
  set(MCU IMXRT1062_T41)
  add_definitions(
    -D__FMU_R_V2_BETA__
  )
else()
  # FMU-R-V1
  set(MCU MK66FX1M0)
  add_definitions(
    -D__FMU_R_V1__
  )
endif()
# Grab the processor and set up definitions and compile options
include(${mcu_support_SOURCE_DIR}/cmake/config_mcu.cmake)
configMcu(${MCU} ${mcu_support_SOURCE_DIR})
# Define larger Serial buffers
if (FMU STREQUAL "V2" OR FMU STREQUAL "V2-BETA")
  add_definitions(
    -DSERIAL2_RX_BUFFER_SIZE=1024
    -DSERIAL3_RX_BUFFER_SIZE=1024
    -DSERIAL3_TX_BUFFER_SIZE=1024
    -DSERIAL4_RX_BUFFER_SIZE=1024
    -DSERIAL4_TX_BUFFER_SIZE=1024
    -DSERIAL5_RX_BUFFER_SIZE=1024
    -DSERIAL5_TX_BUFFER_SIZE=1024
    -DSERIAL7_RX_BUFFER_SIZE=1024
    -DSERIAL7_TX_BUFFER_SIZE=1024
  )
else()
  add_definitions(
    -DSERIAL2_RX_BUFFER_SIZE=1024
    -DSERIAL3_RX_BUFFER_SIZE=1024
    -DSERIAL3_TX_BUFFER_SIZE=1024
    -DSERIAL4_RX_BUFFER_SIZE=1024
    -DSERIAL4_TX_BUFFER_SIZE=1024
  )
endif()
# nanopb
set(NANOPB_SRC_ROOT_FOLDER "/usr/local/nanopb")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${NANOPB_SRC_ROOT_FOLDER}/extra)
find_package(Nanopb REQUIRED)
include_directories(${NANOPB_INCLUDE_DIRS})
if (FMU STREQUAL "V2")
  # FMU-R-V2
  NANOPB_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ../common/datalog_fmu_v2.proto)
elseif(FMU STREQUAL "V2-BETA")
  # FMU-R-V2-BETA
  NANOPB_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ../common/datalog_fmu_v2_beta.proto)
else()
  # FMU-R-V1
  NANOPB_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ../common/datalog_fmu_v1.proto)
endif()
# Include directories
include_directories(${CMAKE_CURRENT_BINARY_DIR})
# Fetch dependencies
include(FetchContent)
FetchContent_Declare(
  core
  GIT_REPOSITORY https://github.com/bolderflight/core.git
  GIT_TAG v3.0.2
)
FetchContent_MakeAvailable(core)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://github.com/bolderflight/eigen.git
  GIT_TAG v3.0.2
)
FetchContent_MakeAvailable(eigen)
FetchContent_Declare(
  mpu9250
  GIT_REPOSITORY https://github.com/bolderflight/mpu9250.git
  GIT_TAG v5.6.0
)
FetchContent_MakeAvailable(mpu9250)
FetchContent_Declare(
  vector_nav
  GIT_REPOSITORY https://github.com/bolderflight/vector_nav.git
  GIT_TAG v3.1.0
)
FetchContent_MakeAvailable(vector_nav)
FetchContent_Declare(
  bme280
  GIT_REPOSITORY https://github.com/bolderflight/bme280.git
  GIT_TAG v5.1.0
)
FetchContent_MakeAvailable(bme280)
FetchContent_Declare(
  ams5915
  GIT_REPOSITORY https://github.com/bolderflight/ams5915.git
  GIT_TAG v5.1.0
)
FetchContent_MakeAvailable(ams5915)
FetchContent_Declare(
  ubx
  GIT_REPOSITORY https://github.com/bolderflight/ublox.git
  GIT_TAG v5.1.0
)
FetchContent_MakeAvailable(ubx)
FetchContent_Declare(
  sbus
  GIT_REPOSITORY https://github.com/bolderflight/sbus.git
  GIT_TAG v7.0.0
)
FetchContent_MakeAvailable(sbus)
FetchContent_Declare(
  pwm
  GIT_REPOSITORY https://github.com/bolderflight/pwm.git
  GIT_TAG v6.0.1
)
FetchContent_MakeAvailable(pwm)
FetchContent_Declare(
  mavlink
  GIT_REPOSITORY https://github.com/bolderflight/mavlink.git
  GIT_TAG v4.0.1
)
FetchContent_MakeAvailable(mavlink)
FetchContent_Declare(
  airdata
  GIT_REPOSITORY https://github.com/bolderflight/airdata.git
  GIT_TAG v3.1.0
)
FetchContent_MakeAvailable(airdata)
FetchContent_Declare(
  navigation
  GIT_REPOSITORY https://github.com/bolderflight/navigation.git
  GIT_TAG v4.0.1
)
FetchContent_MakeAvailable(navigation)
FetchContent_Declare(
  statistics
  GIT_REPOSITORY https://github.com/bolderflight/statistics.git
  GIT_TAG v4.0.3
)
FetchContent_MakeAvailable(statistics)
FetchContent_Declare(
  logger
  GIT_REPOSITORY https://github.com/bolderflight/logger.git
  GIT_TAG v2.1.0
)
FetchContent_MakeAvailable(logger)
FetchContent_Declare(
  control
  GIT_REPOSITORY https://github.com/bolderflight/control.git
  GIT_TAG v3.0.0
)
FetchContent_MakeAvailable(control)
FetchContent_Declare(
  excitation
  GIT_REPOSITORY https://github.com/bolderflight/excitation.git
  GIT_TAG v3.0.0
)
FetchContent_MakeAvailable(excitation)
FetchContent_Declare(
  filter
  GIT_REPOSITORY https://github.com/bolderflight/filter.git
  GIT_TAG v3.1.0
)
FetchContent_MakeAvailable(filter)
FetchContent_Declare(
  polytools
  GIT_REPOSITORY https://github.com/bolderflight/polytools.git
  GIT_TAG v4.0.1
)
FetchContent_MakeAvailable(polytools)
FetchContent_Declare(
  checksum
  GIT_REPOSITORY https://github.com/bolderflight/checksum.git
  GIT_TAG v3.1.1
)
FetchContent_MakeAvailable(checksum)
FetchContent_Declare(
  framing
  GIT_REPOSITORY https://github.com/bolderflight/framing.git
  GIT_TAG v3.1.1
)
FetchContent_MakeAvailable(framing)
FetchContent_Declare(
  ms4525do
  GIT_REPOSITORY https://github.com/bolderflight/ms4525do.git
  GIT_TAG v1.0.0
)
FetchContent_MakeAvailable(ms4525do)
# Add the executable
add_executable(flight
  include/flight/hardware_defs.h
  include/flight/global_defs.h
  include/flight/msg.h
  include/flight/config.h
  include/flight/sensors.h
  include/flight/mpu9250_impl.h
  include/flight/bme280_impl.h
  include/flight/vector_nav_impl.h
  include/flight/ams5915_impl.h
  include/flight/sbus_rx_impl.h
  include/flight/ubx_impl.h
  include/flight/effectors.h
  include/flight/sys.h
  include/flight/vms.h
  include/flight/analog.h
  include/flight/telem.h
  include/flight/airdata_est.h
  include/flight/datalog.h
  include/flight/bfs_ekf.h
  include/flight/ms4525do_impl.h
  flight/flight.cc
  flight/config.cc
  flight/msg.cc
  flight/sensors.cc
  flight/mpu9250_impl.cc
  flight/bme280_impl.cc
  flight/vector_nav_impl.cc
  flight/ams5915_impl.cc
  flight/sbus_rx_impl.cc
  flight/ubx_impl.cc
  flight/effectors.cc
  flight/sys.cc
  flight/vms.cc
  flight/analog.cc
  flight/telem.cc
  flight/airdata_est.cc
  flight/datalog.cc
  flight/bfs_ekf.cc
  flight/ms4525do_impl.cc
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)
if (FMU STREQUAL "V2")
  # FMU-R-V2
  target_sources(flight
    PUBLIC
      include/flight/battery.h
      flight/battery.cc
  )
endif()
# Setup autocode
if (DEFINED AUTOCODE)
  include_directories(autocode/${AUTOCODE}_ert_rtw)
  target_sources(flight
    PUBLIC
      autocode/${AUTOCODE}_ert_rtw/autocode.cpp
      autocode/${AUTOCODE}_ert_rtw/autocode.h
  )
  add_definitions(-D__AUTOCODE__)
endif()
# Add the includes
target_include_directories(flight PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Link libraries to the executable
target_link_libraries(flight
  PRIVATE
    core
    eigen
    mpu9250
    vector_nav
    bme280
    ams5915
    ubx
    sbus
    pwm
    mavlink
    airdata
    navigation
    statistics
    logger
    control
    excitation
    filter
    polytools
    checksum
    framing
    ms4525do
)
# Need std::variant and std::optional
target_compile_options(flight PUBLIC -std=c++20)
# Add hex and upload targets
include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
FlashMcu(flight ${MCU} ${mcu_support_SOURCE_DIR})
