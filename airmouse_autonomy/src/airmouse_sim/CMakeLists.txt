cmake_minimum_required(VERSION 3.8)
project(airmouse_sim)

find_package(ament_cmake REQUIRED)

# Data-only package: SDF worlds + models + Python launch files.
install(DIRECTORY worlds models launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
