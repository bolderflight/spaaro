cmake_minimum_required(VERSION 3.13)
# Project information
project(Mat-Converter
	VERSION 2.2.0
	DESCRIPTION "Software to convert data logs to MATLAB v4 format"
	LANGUAGES CXX
)
# FMU version
if (DEFINED FMU)
	string(TOUPPER ${FMU} FMU)
endif()
if (FMU STREQUAL "V2")
	# FMU-R-V2
	add_definitions(
		-D__FMU_R_V2__
	)
elseif(FMU STREQUAL "V2-BETA")
	# FMU-R-V2-BETA
	add_definitions(
		-D__FMU_R_V2_BETA__
	)
else()
	# FMU-R-V1
	add_definitions(
		-D__FMU_R_V1__
	)
endif()
# Protobuf
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
if (FMU STREQUAL "V2")
	# FMU-R-V2
	protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ../common/datalog_fmu_v2.proto)
elseif(FMU STREQUAL "V2-BETA")
	# FMU-R-V2-BETA
	protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ../common/datalog_fmu_v2_beta.proto)
else()
	# FMU-R-V1
	protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ../common/datalog_fmu_v1.proto)
endif()
# Fetch dependencies
include(FetchContent)
FetchContent_Declare(
	framing
	GIT_REPOSITORY 	https://github.com/bolderflight/framing.git
	GIT_TAG v2.0.0
)
FetchContent_MakeAvailable(framing)
FetchContent_Declare(
	mat_v4
	GIT_REPOSITORY 	https://github.com/bolderflight/mat_v4.git
	GIT_TAG v2.0.0
)
FetchContent_MakeAvailable(mat_v4)
FetchContent_Declare(
	eigen
	GIT_REPOSITORY 	https://github.com/bolderflight/eigen.git
	GIT_TAG v1.0.0
)
FetchContent_MakeAvailable(eigen)
# Add the executable
add_executable(mat_converter 
	mat_converter/mat_converter.cc
	${PROTO_SRCS} 
	${PROTO_HDRS}
)
# Link libraries to the executable
target_link_libraries(mat_converter
	PRIVATE 
		framing
		mat_v4
		eigen
		${Protobuf_LIBRARIES}
)
