cmake_minimum_required(VERSION 3.14)
# Project information
project(Mat-Converter
  VERSION 3.0.0
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
elseif(FMU STREQUAL "MINI-V1")
  # FMU-R-V2-BETA
  add_definitions(
    -D__FMU_R_MINI_V1__
  )
else()
  # FMU-R-V1
  add_definitions(
    -D__FMU_R_V1__
  )
endif()
# Fetch dependencies
include(FetchContent)
FetchContent_Declare(
  framing
  GIT_REPOSITORY 	https://github.com/bolderflight/framing.git
  GIT_TAG v3.2.0
)
FetchContent_MakeAvailable(framing)
FetchContent_Declare(
  mat_v4
  GIT_REPOSITORY 	https://github.com/bolderflight/mat_v4.git
  GIT_TAG v2.0.1
)
FetchContent_MakeAvailable(mat_v4)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY 	https://github.com/bolderflight/eigen.git
  GIT_TAG v3.0.2
)
FetchContent_MakeAvailable(eigen)
FetchContent_Declare(
  units
  GIT_REPOSITORY 	https://github.com/bolderflight/units.git
  GIT_TAG v4.1.1
)
FetchContent_MakeAvailable(units)
# Add the executable
add_executable(mat_converter 
  mat_converter/mat_converter.cc
)
# Link libraries to the executable
target_link_libraries(mat_converter
  PRIVATE 
    framing
    mat_v4
    eigen
    units
)
