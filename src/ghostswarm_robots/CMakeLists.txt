cmake_minimum_required(VERSION 3.8)
project(ghostswarm_robots)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Install Python modules
install(DIRECTORY
  scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/
)

# Install models
install(DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF
install(DIRECTORY
  urdf/
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz config
install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
