cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(ament_cmake REQUIRED)

# Installiere die Verzeichnisse
install(
  DIRECTORY
    launch
    urdf
    models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()