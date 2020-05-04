cmake_minimum_required(VERSION 2.8.3)
project(odrive_cpp_sdk)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)
include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})

add_library(odrive_cpp_sdk src/robot.cpp src/robot.h src/motor.cpp src/motor.h src/ODrive.cpp src/ODrive.h src/channel.cpp src/channel.h)

target_link_libraries(odrive_cpp_sdk ${LIBUSB1_LIBRARIES})

add_executable(odrive_test
  src/main.cpp
)

target_link_libraries(odrive_test odrive_cpp_sdk)
