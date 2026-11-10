cmake_minimum_required(VERSION 3.8)
project(airmouse_sensors)

find_package(ament_cmake REQUIRED)

# Data-only package: bridge YAML + Python launch files.
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
