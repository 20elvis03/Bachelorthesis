cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  models
  scripts
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/autonomous.py
  scripts/manually.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()