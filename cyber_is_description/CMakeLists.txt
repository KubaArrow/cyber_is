cmake_minimum_required(VERSION 3.5)
project(cyber_is_description)

find_package(ament_cmake REQUIRED)

# Install shared resources (URDF, launch files, RViz config, worlds)
install(DIRECTORY
  launch
  urdf
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
