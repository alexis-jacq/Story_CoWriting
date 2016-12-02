cmake_minimum_required(VERSION 2.8.3)
project(story_cowriting)

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
  CATKIN_DEPENDS openface_tracker
)

install(PROGRAMS
  nodes/nao_actions.py
  nodes/main_activity.py
  nodes/interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/story_cowriting.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
