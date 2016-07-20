cmake_minimum_required(VERSION 2.8.3)
project(mutual_modelling)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

include_directories(${catkin_INCLUDE_DIRS})

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS attention_tracker
)

install(PROGRAMS
  nodes/cowriter_agents_info.py
  nodes/cowriter_mutual_modelling.py
  nodes/nao_actions.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/mutual_modelling_for_cowriter.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
