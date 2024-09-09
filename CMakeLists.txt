cmake_minimum_required(VERSION 2.8)

project(ARM_VO)

# SET(CMAKE_BUILD_TYPE Release)

if (${CROSS_ENABLE} MATCHES "ON")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O3 -std=c++11 -mfpu=neon -mfloat-abi=hard -funroll-loops -Ofast")
else()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O3 -std=c++11 -msse4.2")
endif()

find_package(OpenCV 4 REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found.")
endif()

include_directories(include)

file(GLOB SOURCES "src/*.cpp")
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

FIND_PACKAGE(OpenMP)
if(OPENMP_FOUND)
  message("OPENMP FOUND")
  target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()