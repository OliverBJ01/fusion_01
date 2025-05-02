cmake_minimum_required(VERSION 3.8)
project(lidar_subscriber_pkg)

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add the executable, linking it to the source file
add_executable(lidar_subscriber_node src/lidar_subscriber.cpp)

# Link the executable against the required libraries
ament_target_dependencies(lidar_subscriber_node
  rclcpp
  sensor_msgs
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Optional: Install rviz config files
# install(DIRECTORY
#  rviz
#  DESTINATION share/${PROJECT_NAME}
# )

# Install the executable to the install/lidar_subscriber_pkg/lib directory
install(TARGETS lidar_subscriber_node
  DESTINATION lib/${PROJECT_NAME})

# Standard ROS 2 package finishing lines
ament_package()

