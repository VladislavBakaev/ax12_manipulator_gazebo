cmake_minimum_required(VERSION 3.0.2)
project(angle_ax12_model_pkg)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  urdf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES angle_ax12_model_pkg
#  CATKIN_DEPENDS roscpp rospy urdf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS nodes/gripper_control.py
                      nodes/pub.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
  launch/angle_arm_ax12_load.launch
  launch/angle_robot_ax12_model.launch
  launch/angle_robot_ax12.launch
  launch/angle_robot_nodes.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
