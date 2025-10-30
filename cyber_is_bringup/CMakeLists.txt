cmake_minimum_required(VERSION 3.8)
project(cyber_is_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Optionally install other resources if needed by users
install(DIRECTORY
  urdf
  services
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
