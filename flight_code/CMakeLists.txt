cmake_minimum_required(VERSION 3.13)
# Using the MK66FX1M0 microcontroller
set(MCU MK66FX1M0)
# Setting up the toolchain
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/cortex.cmake")
# Project information
project(Flight
	VERSION 2.0.4
	DESCRIPTION "Flight software skeleton"
	LANGUAGES C CXX
)
# Export project info
configure_file(${CMAKE_SOURCE_DIR}/cmake/version.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/version.h)
# Grab the processor and set up definitions and compile options
include(${CMAKE_SOURCE_DIR}/cmake/config_mcu.cmake)
configMcu(${MCU})
# Define a larger Serial 2 RX buffer
add_definitions(
	-DSERIAL2_RX_BUFFER_SIZE=1024
)
# Define a larger Serial 3 RX buffer
add_definitions(
	-DSERIAL3_RX_BUFFER_SIZE=2048
)
# Define a larger Serial 4 TX buffer
add_definitions(
	-DSERIAL4_TX_BUFFER_SIZE=2048
)
# nanopb
set(NANOPB_SRC_ROOT_FOLDER "/usr/local/nanopb")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${NANOPB_SRC_ROOT_FOLDER}/extra)
find_package(Nanopb REQUIRED)
include_directories(${NANOPB_INCLUDE_DIRS})
NANOPB_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ../common/datalog.proto)
# Include directories
include_directories(${CMAKE_CURRENT_BINARY_DIR})
# Fetch dependencies
include(FetchContent)
FetchContent_Declare(
	core
	GIT_REPOSITORY https://github.com/bolderflight/core.git
	GIT_TAG v2.0.5
)
FetchContent_MakeAvailable(core)
FetchContent_Declare(
	imu
	GIT_REPOSITORY https://github.com/bolderflight/imu.git
	GIT_TAG v2.2.0
)
FetchContent_MakeAvailable(imu)
FetchContent_Declare(
	gnss
	GIT_REPOSITORY https://github.com/bolderflight/gnss.git
	GIT_TAG v2.4.0
)
FetchContent_MakeAvailable(gnss)
FetchContent_Declare(
	pres
	GIT_REPOSITORY https://github.com/bolderflight/pres.git
	GIT_TAG v1.2.0
)
FetchContent_MakeAvailable(pres)
FetchContent_Declare(
	inceptor
	GIT_REPOSITORY https://github.com/bolderflight/inceptor.git
	GIT_TAG v2.2.0
)
FetchContent_MakeAvailable(inceptor)
FetchContent_Declare(
	effector
	GIT_REPOSITORY https://github.com/bolderflight/effector.git
	GIT_TAG v6.1.3
)
FetchContent_MakeAvailable(effector)
FetchContent_Declare(
	navigation
	GIT_REPOSITORY https://github.com/bolderflight/navigation.git
	GIT_TAG v2.0.1
)
FetchContent_MakeAvailable(navigation)
FetchContent_Declare(
	airdata
	GIT_REPOSITORY https://github.com/bolderflight/airdata.git
	GIT_TAG v2.1.0
)
FetchContent_MakeAvailable(airdata)
FetchContent_Declare(
	logger
	GIT_REPOSITORY https://github.com/bolderflight/logger.git
	GIT_TAG v2.0.3
)
FetchContent_MakeAvailable(logger)
FetchContent_Declare(
	framing
	GIT_REPOSITORY https://github.com/bolderflight/framing.git
	GIT_TAG v2.0.0
)
FetchContent_MakeAvailable(framing)
FetchContent_Declare(
	filter
	GIT_REPOSITORY https://github.com/bolderflight/filter.git
	GIT_TAG v2.1.1
)
FetchContent_MakeAvailable(filter)
FetchContent_Declare(
	units
	GIT_REPOSITORY https://github.com/bolderflight/units.git
	GIT_TAG v3.2.0
)
FetchContent_MakeAvailable(units)
FetchContent_Declare(
	mavlink
	GIT_REPOSITORY https://github.com/bolderflight/mavlink.git
	GIT_TAG v3.1.2
)
FetchContent_MakeAvailable(mavlink)
FetchContent_Declare(
	mpu9250
	GIT_REPOSITORY https://github.com/bolderflight/mpu9250.git
	GIT_TAG v4.2.0
)
FetchContent_MakeAvailable(mpu9250)
FetchContent_Declare(
	ublox
	GIT_REPOSITORY https://github.com/bolderflight/ublox.git
	GIT_TAG v3.2.0
)
FetchContent_MakeAvailable(ublox)
FetchContent_Declare(
	ams5915
	GIT_REPOSITORY https://github.com/bolderflight/ams5915.git
	GIT_TAG v4.1.0
)
FetchContent_MakeAvailable(ams5915)
FetchContent_Declare(
	bme280
	GIT_REPOSITORY https://github.com/bolderflight/bme280.git
	GIT_TAG v4.1.0
)
FetchContent_MakeAvailable(bme280)
FetchContent_Declare(
	sbus
	GIT_REPOSITORY https://github.com/bolderflight/sbus.git
	GIT_TAG v4.0.5
)
FetchContent_MakeAvailable(sbus)
FetchContent_Declare(
	pwm
	GIT_REPOSITORY https://github.com/bolderflight/pwm.git
	GIT_TAG v4.0.4
)
FetchContent_MakeAvailable(pwm)
# Add the executable
add_executable(flight
	include/flight/global_defs.h
	include/flight/hardware_defs.h
	include/flight/msg.h
	include/flight/sys.h
	include/flight/sensors.h
	include/flight/effectors.h
	include/flight/nav.h
	include/flight/control.h
	include/flight/datalog.h
	include/flight/telem.h
	flight/flight.cc
	flight/config.cc
	flight/msg.cc
	flight/sys.cc
	flight/sensors.cc
	flight/effectors.cc
	flight/nav.cc
	flight/control.cc
	flight/datalog.cc
	flight/telem.cc
	${PROTO_SRCS}
	${PROTO_HDRS}
)
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
		imu
		gnss
		pres
		inceptor
		effector
		navigation
		airdata
		logger
		framing
		filter
		units
		mavlink
		mpu9250
		ublox
		ams5915
		bme280
		sbus
		pwm
)
# Add hex and upload targets
include(${CMAKE_SOURCE_DIR}/cmake/flash_mcu.cmake)
FlashMcu(flight ${MCU})
