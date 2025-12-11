cmake_minimum_required(VERSION 3.5)
project(cyber_is_navigation)

find_package(ament_cmake REQUIRED)

# Install only ROS 2 launch and config
install(FILES
  launch/start_navigation.launch.py
  launch/start_slam_navigation.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES
  config/nav2_params.yaml
  config/slam_toolbox_online_async.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

ament_package()
