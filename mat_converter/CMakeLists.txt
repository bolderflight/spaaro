cmake_minimum_required(VERSION 3.13)
# Project information
project(Mat-Converter
	VERSION 1.0.4
	DESCRIPTION "Software to convert data logs to MATLAB v4 format"
	LANGUAGES CXX
)
# Protobuf
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ../common/datalog.proto)
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
