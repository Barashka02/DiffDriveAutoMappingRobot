cmake_minimum_required(VERSION 3.8)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(ros_interfaces REQUIRED)


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${behaviortree_cpp_INCLUDE_DIRS}
  ${behaviortree_ros2_INCLUDE_DIRS}
  ${nav2_msgs_INCLUDE_DIRS}
  ${rosidl_typesupport_interface_INCLUDE_DIRS}
)

# Add executables
add_executable(robot_subscriber src/robot_subscriber.cpp)
ament_target_dependencies(robot_subscriber 
  rclcpp 
  sensor_msgs 
  std_msgs 
  nav_msgs   
  behaviortree_cpp
  behaviortree_ros2
  nav2_msgs
  rosidl_typesupport_interface
)


add_executable(executive src/executive.cpp)
ament_target_dependencies(executive
  rclcpp 
  behaviortree_cpp
  behaviortree_ros2
  nav2_msgs
  ros_interfaces
  tf2
  tf2_ros
  rosidl_typesupport_interface
  slam_toolbox
)

add_executable(pose_srv src/get_pose_server.cpp)
ament_target_dependencies(pose_srv
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  ros_interfaces
  nav2_msgs
  nav_msgs
)

# Install executables
install(TARGETS
  robot_subscriber
  executive
  pose_srv
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY src DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
