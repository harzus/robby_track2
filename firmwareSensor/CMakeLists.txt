cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

generate_arduino_firmware(rt9axesSensor
  SRCS rt9axesSensor.cpp arduino_library_nine_axes_motion/NineAxesMotion.cpp arduino_library_nine_axes_motion/utility/BNO055.c ${ROS_LIB_DIR}/time.cpp
  BOARD uno
  PORT /dev/ttyACM1
)

