cmake_minimum_required(VERSION 3.8)
project(bjo_test)

# Require C++ 14 or newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Find rclcpp for building ROS 2 nodes
find_package(std_msgs REQUIRED)  # If you use standard message types
find_package(rosidl_default_generators REQUIRED) # Add for custom msgs if

# Add your executable
add_executable(main_node main.cpp)  # Replace main.cpp with the name of your file

# Link against ROS 2 libraries
ament_target_dependencies(main_node
        rclcpp
        std_msgs   # Remove if not using std_msgs
)

# Install the target (optional, for packaging/executing)
install(TARGETS 
	main_node
DESTINATION lib/${PROJECT_NAME})

# Ament package macros for ROS 2
ament_package()
