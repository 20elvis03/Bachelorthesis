cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

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
  scripts/manually.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME manually
)

install(PROGRAMS
  scripts/autonomous.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME autonomous
)

install(PROGRAMS
  scripts/emergency_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME emergency_controller
)

ament_package()