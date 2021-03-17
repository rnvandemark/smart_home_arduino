cmake_minimum_required(VERSION 2.8.3)
project(smart_home_arduino)

find_package(catkin REQUIRED COMPONENTS
	rosserial_arduino
	rosserial_client
	smart_home_msgs
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS
	rosserial_arduino
	rosserial_client
	std_msgs
)

rosserial_generate_ros_lib(
	PACKAGE rosserial_arduino
	SCRIPT make_libraries.py
)

rosserial_configure_client(
	DIRECTORY firmware
	TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware traffic_light_runner ALL)
rosserial_add_client_target(firmware traffic_light_runner-upload)
