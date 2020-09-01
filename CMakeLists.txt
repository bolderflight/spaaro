cmake_minimum_required(VERSION 3.13)
# Using the MK66FX1M0 microcontroller
set(MCU MK66FX1M0)
# Setting up the toolchain
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/cortex.cmake")
# Project information
project(FixedWing
	VERSION 1.0.0
	DESCRIPTION "Fixed wing flight software"
	LANGUAGES C CXX
)
# Grab the processor and set up definitions and compile options
include(${CMAKE_SOURCE_DIR}/cmake/config_mcu.cmake)
configMcu(${MCU})
# Define a larger Serial 3 RX buffer
add_definitions(
	-DSERIAL3_RX_BUFFER_SIZE=1024
)
# nanopb
set(NANOPB_SRC_ROOT_FOLDER "/usr/local/nanopb")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${NANOPB_SRC_ROOT_FOLDER}/extra)
find_package(Nanopb REQUIRED)
include_directories(${NANOPB_INCLUDE_DIRS})
NANOPB_GENERATE_CPP(PROTO_SRCS PROTO_HDRS fixed_wing/datalog.proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
# Fetch dependencies
include(FetchContent)
FetchContent_Declare(
	core
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/core.git
)
FetchContent_MakeAvailable(core)
FetchContent_Declare(
	eigen
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/eigen.git
)
FetchContent_MakeAvailable(eigen)
FetchContent_Declare(
	mpu9250
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/mpu9250.git
)
FetchContent_MakeAvailable(mpu9250)
FetchContent_Declare(
	ublox
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/ublox.git
)
FetchContent_MakeAvailable(ublox)
FetchContent_Declare(
	navigation
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/navigation.git
)
FetchContent_MakeAvailable(navigation)
FetchContent_Declare(
	ams5915
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/ams5915.git
)
FetchContent_MakeAvailable(ams5915)
FetchContent_Declare(
	airdata
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/airdata.git
)
FetchContent_MakeAvailable(airdata)
FetchContent_Declare(
	filter
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/filter.git
)
FetchContent_MakeAvailable(filter)
FetchContent_Declare(
	statistics
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/statistics.git
)
FetchContent_MakeAvailable(statistics)
FetchContent_Declare(
	sbus
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/sbus.git
)
FetchContent_MakeAvailable(sbus)
FetchContent_Declare(
	polytools
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/polytools.git
)
FetchContent_MakeAvailable(polytools)
FetchContent_Declare(
	pwm
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/pwm.git
)
FetchContent_MakeAvailable(pwm)
FetchContent_Declare(
	actuator
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/actuator.git
)
FetchContent_MakeAvailable(actuator)
FetchContent_Declare(
	logger
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/logger.git
)
FetchContent_MakeAvailable(logger)
FetchContent_Declare(
	framing
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/framing.git
)
FetchContent_MakeAvailable(framing)
# Add the executable
add_executable(flight
	fixed_wing/flight.cc
	fixed_wing/print_msg.cc
	fixed_wing/inceptor.cc
	fixed_wing/ins.cc
	fixed_wing/airdata.cc
	fixed_wing/control.cc
	fixed_wing/effector.cc
	fixed_wing/datalog.cc
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
)
# Add hex and upload targets
include(${CMAKE_SOURCE_DIR}/cmake/flash_mcu.cmake)
FlashMcu(flight ${MCU})
