cmake_minimum_required(VERSION 3.0.2)
project(rotf3)
add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_msgs
  gazebo_dev
  message_generation
  urdf
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
)

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES omni_controller gazebo_ros_utils

  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs 
  nav_msgs 
  tf 
  gazebo_ros
  message_runtime
  gazebo_msgs
  rospy
  nav_msgs
  urdf
  tf
)

add_library(gazebo_ros_utils src/gazebo_ros_utils.cpp)
target_link_libraries(gazebo_ros_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(omni_controller src/omni_controller.cpp)
target_link_libraries(omni_controller gazebo_ros_utils ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_install_python(PROGRAMS 
  scripts/get_odom.py 
  scripts/target_pub.py
  scripts/pid.py
  scripts/pseudo_encoder.py
  scripts/read_from_arduino.py
  scripts/teleop_rotf3.py
  scripts/loop_cmd_vel.py
  scripts/teleop_key.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



install(TARGETS omni_controller gazebo_ros_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

