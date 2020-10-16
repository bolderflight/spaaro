cmake_minimum_required(VERSION 3.13)
# Using the MK66FX1M0 microcontroller
set(MCU MK66FX1M0)
# Setting up the toolchain
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/cortex.cmake")
# Project information
project(Flight
	VERSION 1.0.1
	DESCRIPTION "Flight software skeleton"
	LANGUAGES C CXX
)
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
NANOPB_GENERATE_CPP(PROTO_SRCS PROTO_HDRS flight/datalog.proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
# Fetch dependencies
include(FetchContent)
FetchContent_Declare(
	core
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/core.git
	GIT_TAG v1.0.1
)
FetchContent_MakeAvailable(core)
FetchContent_Declare(
	eigen
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/eigen.git
	GIT_TAG v1.0.0
)
FetchContent_MakeAvailable(eigen)
FetchContent_Declare(
	mpu9250
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/mpu9250.git
	GIT_TAG v1.0.1
)
FetchContent_MakeAvailable(mpu9250)
FetchContent_Declare(
	ublox
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/ublox.git
	GIT_TAG v1.2.1
)
FetchContent_MakeAvailable(ublox)
FetchContent_Declare(
	navigation
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/navigation.git
	GIT_TAG v1.2.2
)
FetchContent_MakeAvailable(navigation)
FetchContent_Declare(
	ams5915
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/ams5915.git
	GIT_TAG v1.0.1
)
FetchContent_MakeAvailable(ams5915)
FetchContent_Declare(
	airdata
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/airdata.git
	GIT_TAG v1.0.1
)
FetchContent_MakeAvailable(airdata)
FetchContent_Declare(
	filter
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/filter.git
	GIT_TAG v1.0.1
)
FetchContent_MakeAvailable(filter)
FetchContent_Declare(
	statistics
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/statistics.git
	GIT_TAG v1.0.1
)
FetchContent_MakeAvailable(statistics)
FetchContent_Declare(
	sbus
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/sbus.git
	GIT_TAG v1.0.2
)
FetchContent_MakeAvailable(sbus)
FetchContent_Declare(
	polytools
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/polytools.git
	GIT_TAG v1.0.2
)
FetchContent_MakeAvailable(polytools)
FetchContent_Declare(
	pwm
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/pwm.git
	GIT_TAG v1.0.1
)
FetchContent_MakeAvailable(pwm)
FetchContent_Declare(
	actuator
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/actuator.git
	GIT_TAG v1.0.1
)
FetchContent_MakeAvailable(actuator)
FetchContent_Declare(
	logger
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/logger.git
	GIT_TAG v1.0.2
)
FetchContent_MakeAvailable(logger)
FetchContent_Declare(
	framing
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/framing.git
	GIT_TAG v1.1.1
)
FetchContent_MakeAvailable(framing)
FetchContent_Declare(
	mavlink
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/mavlink.git
	GIT_TAG v1.0.1
)
FetchContent_MakeAvailable(mavlink)
# Add the executable
add_executable(flight
	flight/flight.cc
	flight/print_msg.cc
	flight/inceptor.cc
	flight/ins.cc
	flight/airdata.cc
	flight/control.cc
	flight/effector.cc
	flight/status.cc
	flight/datalog.cc
	flight/telemetry.cc
	${PROTO_SRCS}
	${PROTO_HDRS}
)
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
		ublox
		navigation
		ams5915
		airdata
		filter
		statistics
		sbus
		polytools
		pwm
		actuator
		logger
		framing
		mavlink
)
# Add hex and upload targets
include(${CMAKE_SOURCE_DIR}/cmake/flash_mcu.cmake)
FlashMcu(flight ${MCU})
