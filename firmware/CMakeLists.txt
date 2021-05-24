cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_STANDARD 11)

message(STATUS "ROS_LIB_DIR='${ROS_LIB_DIR}'")
include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

set(mega.build.mcu atmega328p)
set(mega.upload.speed 115200)
generate_arduino_firmware(traffic_light_runner
	SRCS TrafficLightRunner.cpp ${ROS_LIB_DIR}/time.cpp
	BOARD uno
	PORT /dev/ttyUSB0
	PROGRAMMER avrispmkii
	SERIAL picocom -b 115200 -l @SERIAL_PORT@
)
