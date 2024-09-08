cmake_minimum_required(VERSION 3.14)
include(FetchContent)
FetchContent_Declare(
  mcu_support
  GIT_REPOSITORY https://github.com/bolderflight/mcu-support.git
  GIT_TAG v1.1.0
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
if (FMU STREQUAL "MINI-V1")
  # FMU-R-mini
  set(MCU IMXRT1062_MMOD)
  add_definitions(
    -D__FMU_R_MINI_V1__
  )
elseif (FMU STREQUAL "V2")
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

# determin SBUS radio module
if (DEFINED SBUS_SRC)
  string(TOUPPER ${SBUS_SRC} SBUS_SRC)
endif()

# determine config.cc based on vehicle type
if (DEFINED VEHICLE)
	string(TOUPPER ${VEHICLE} VEHICLE)
endif()
if (VEHICLE STREQUAL "ALE")
	message("Using ALE config.cc")
	set(CONFIG_CC config_ale.cc)
elseif (VEHICLE STREQUAL "MALT")
	message("Using MALT config.cc")
	set(CONFIG_CC config_malt.cc)
elseif (VEHICLE STREQUAL "SUPER")
	message("Using SUPER config.cc")
	set(CONFIG_CC config_super.cc)
elseif (VEHICLE STREQUAL "LAMBU")
  message("Using LAMBU config.cc")
	set(CONFIG_CC config_lambu.cc)
else()
	message("Using default config.cc")
	set(CONFIG_CC config.cc)
endif()

# Grab the processor and set up definitions and compile options
include(${mcu_support_SOURCE_DIR}/cmake/config_mcu.cmake)
configMcu(${MCU} ${mcu_support_SOURCE_DIR})
# Define larger Serial buffers
if (FMU STREQUAL "MINI-V1" OR FMU STREQUAL "V2" OR FMU STREQUAL "V2-BETA")
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
# Include directories
include_directories(${CMAKE_CURRENT_BINARY_DIR})
# Fetch dependencies
include(FetchContent)
FetchContent_Declare(
  core
  GIT_REPOSITORY https://github.com/bolderflight/core.git
  GIT_TAG v3.1.3
)
FetchContent_MakeAvailable(core)
FetchContent_Declare(
  invensense_imu
  GIT_REPOSITORY https://github.com/bolderflight/invensense-imu.git
  GIT_TAG v6.0.3
)
FetchContent_MakeAvailable(invensense_imu)
FetchContent_Declare(
  bme280
  GIT_REPOSITORY https://github.com/bolderflight/bme280.git
  GIT_TAG v5.2.6
)
FetchContent_MakeAvailable(bme280)
FetchContent_Declare(
  bmp3
  GIT_REPOSITORY https://github.com/bolderflight/bmp3.git
  GIT_TAG v1.1.6
)
FetchContent_MakeAvailable(bmp3)
FetchContent_Declare(
  lis3mdl
  GIT_REPOSITORY https://github.com/bolderflight/lis3mdl.git
  GIT_TAG v1.1.5
)
FetchContent_MakeAvailable(lis3mdl)
FetchContent_Declare(
  ams5915
  GIT_REPOSITORY https://github.com/bolderflight/ams5915.git
  GIT_TAG v6.0.3
)
FetchContent_MakeAvailable(ams5915)
FetchContent_Declare(
  ubx
  GIT_REPOSITORY https://github.com/bolderflight/ublox.git
  GIT_TAG v6.0.6
)
FetchContent_MakeAvailable(ubx)
FetchContent_Declare(
  ainstein_usd1
  GIT_REPOSITORY https://github.com/bolderflight/ainstein-usd1.git
  GIT_TAG v1.1.3
)
FetchContent_MakeAvailable(ainstein_usd1)
if (SBUS_SRC STREQUAL "XBEE")
  message("Fetching XBEE sbus module")
  FetchContent_Declare(
    sbus
    GIT_REPOSITORY https://github.com/TL-4319/sbus
    GIT_TAG v1.0.0
    )
else()
  FetchContent_Declare(
    sbus
    GIT_REPOSITORY https://github.com/bolderflight/sbus.git
    GIT_TAG v8.1.3
  )
endif()
FetchContent_MakeAvailable(sbus)
FetchContent_Declare(
  vector_nav
  GIT_REPOSITORY https://github.com/bolderflight/vector-nav.git
  GIT_TAG v4.0.4
)
FetchContent_MakeAvailable(vector_nav)
FetchContent_Declare(
  pwm
  GIT_REPOSITORY https://github.com/bolderflight/pwm.git
  GIT_TAG v6.0.2
)
FetchContent_MakeAvailable(pwm)
FetchContent_Declare(
  mavlink
  #GIT_REPOSITORY https://github.com/bolderflight/mavlink.git
  #GIT_TAG v4.8.10
  GIT_REPOSITORY https://github.com/TL-4319/mavlink.git
  GIT_TAG v1.0.2
  #SOURCE_DIR /home/tuan/Projects/mavlink
)
FetchContent_MakeAvailable(mavlink)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://github.com/bolderflight/eigen.git
  GIT_TAG v3.0.2
)
FetchContent_MakeAvailable(eigen)
FetchContent_Declare(
  statistics
  GIT_REPOSITORY https://github.com/bolderflight/statistics.git
  GIT_TAG v4.0.4
)
FetchContent_MakeAvailable(statistics)
FetchContent_Declare(
  logger
  GIT_REPOSITORY https://github.com/bolderflight/logger.git
  GIT_TAG v2.1.0
)
FetchContent_MakeAvailable(logger)
FetchContent_Declare(
  framing
  GIT_REPOSITORY https://github.com/bolderflight/framing.git
  GIT_TAG v3.1.1
)
FetchContent_MakeAvailable(framing)
FetchContent_Declare(
  navigation
  #GIT_REPOSITORY https://github.com/bolderflight/navigation.git
  #GIT_TAG v4.0.1
  GIT_REPOSITORY https://github.com/TL-4319/navigation.git
  GIT_TAG f870deb4303e73fa4c32b54126e7402174ea668d
  #SOURCE_DIR /home/tuan/Projects/navigation
)
FetchContent_MakeAvailable(navigation)
FetchContent_Declare(
  airdata
  GIT_REPOSITORY https://github.com/bolderflight/airdata.git
  GIT_TAG v3.1.0
)
FetchContent_MakeAvailable(airdata)
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
  polytools
  GIT_REPOSITORY https://github.com/bolderflight/polytools.git
  GIT_TAG v4.0.2
)
FetchContent_MakeAvailable(polytools)
FetchContent_Declare(
  units
  GIT_REPOSITORY https://github.com/bolderflight/units.git
  GIT_TAG v4.1.1
)
FetchContent_MakeAvailable(units)
FetchContent_Declare(
  filter
  GIT_REPOSITORY https://github.com/bolderflight/filter.git
  GIT_TAG v3.2.0
)
FetchContent_MakeAvailable(filter)
FetchContent_Declare(
  imap
  GIT_REPOSITORY https://github.com/bolderflight/imap.git
  GIT_TAG v2.0.2
)
FetchContent_MakeAvailable(imap)
FetchContent_Declare(
  matek3901
  GIT_REPOSITORY https://github.com/TL-4319/matek3901
  GIT_TAG v1.1.0
)
FetchContent_MakeAvailable(matek3901)
# Add the executable
add_executable(flight
  include/global_defs.h
  include/hardware_defs.h
  include/flight/msg.h
  include/flight/config.h
  include/flight/sys.h
  include/flight/sensors.h
  include/flight/adc.h
  include/flight/bfs-ins.h
  include/flight/aux-ins.h
  include/flight/datalog.h
  include/flight/effectors.h
  include/flight/telem.h
  include/flight/vms.h
  include/drivers/fmu.h
  include/drivers/spaaro-lis3mdl.h
  include/drivers/spaaro-ams5915.h
  include/drivers/spaaro-ublox.h
  include/drivers/spaaro-ainstein-usd1.h
  include/drivers/spaaro-sbus.h
  include/drivers/analog.h
  include/drivers/spaaro-vector-nav.h
  include/drivers/spaaro-matek3901.h
  flight/flight.cc
  flight/msg.cc
  flight/${CONFIG_CC}
  flight/sys.cc
  flight/sensors.cc
  flight/adc.cc
  flight/bfs-ins.cc
  flight/aux-ins.cc
  flight/datalog.cc
  flight/effectors.cc
  flight/telem.cc
  flight/vms.cc
  drivers/fmu.cc
  drivers/spaaro-lis3mdl.cc
  drivers/spaaro-ams5915.cc
  drivers/spaaro-ublox.cc
  drivers/spaaro-ainstein-usd1.cc
  drivers/spaaro-sbus.cc
  drivers/analog.cc
  drivers/spaaro-vector-nav.cc
  drivers/spaaro-matek3901.cc
)
if (FMU STREQUAL "MINI-V1" OR FMU STREQUAL "V2")
  # FMU-R mini V1 | FMU-R-V2
  target_sources(flight
    PUBLIC
      include/drivers/power-module.h
      drivers/power-module.cc
  )
endif()

# Setup autocode
if (DEFINED AUTOCODE)
	include_directories(autocode/${AUTOCODE}_ert_rtw)
	target_sources(flight
		PUBLIC
			autocode/${AUTOCODE}_ert_rtw/autocode.cpp
			autocode/${AUTOCODE}_ert_rtw/autocode.h
			#autocode/${AUTOCODE}_ert_rtw/ale_speed_controller.cpp
			#autocode/${AUTOCODE}_ert_rtw/ale_speed_controller.h
			#autocode/${AUTOCODE}_ert_rtw/rt_roundf.cpp
			#autocode/${AUTOCODE}_ert_rtw/rt_roundf.h
			autocode/${AUTOCODE}_ert_rtw/rtwtypes.h
      #autocode/${AUTOCODE}_ert_rtw/zero_crossing_types.h
      autocode/${AUTOCODE}_ert_rtw/rtmodel.h
			#autocode/${AUTOCODE}_ert_rtw/const_params.cpp
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
    mavlink
    statistics
    invensense_imu
    bme280
    bmp3
    lis3mdl
    ams5915
    ubx
    ainstein_usd1
    navigation
    airdata
    logger
    framing
    filter
    units
    sbus
    control
    excitation
    polytools
    vector_nav
    pwm
    imap
    matek3901
)
target_compile_options(flight PUBLIC -std=c++20 -O3)
# Add hex and upload targets
include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
FlashMcu(flight ${MCU} ${mcu_support_SOURCE_DIR})
