cmake_minimum_required(VERSION 3.13)
# Using the MK66FX1M0 microcontroller
set(MCU MK66FX1M0)
# Setting up the toolchain
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/cortex.cmake")
# Project information
project(Flight
	VERSION 2.0.0
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
# Fetch dependencies
include(FetchContent)
FetchContent_Declare(
	core
	GIT_REPOSITORY https://github.com/bolderflight/core.git
	GIT_TAG v2.0.4
)
FetchContent_MakeAvailable(core)
# Add the executable
add_executable(flight
	include/flight/global_defs.h
	include/flight/hardware_defs.h
	include/flight/msg.h
	flight/msg.cc
	flight/flight.cc
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
)
# Add hex and upload targets
include(${CMAKE_SOURCE_DIR}/cmake/flash_mcu.cmake)
FlashMcu(flight ${MCU})
