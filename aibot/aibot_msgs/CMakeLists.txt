cmake_minimum_required(VERSION 2.8.3)
project(aibot_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  )

add_message_files(
  FILES
  ServoCtrl.msg
  MotorCtrl.msg
  LedCtrl.msg
  Oled.msg
  PinMode.msg
  PinValue.msg
  Platform.msg
  Velocities.msg
  PID.msg
  Imu.msg
)



generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)


catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})