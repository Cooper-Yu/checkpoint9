cmake_minimum_required(VERSION 3.8)
project(my_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(attach_shelf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(pre_approach_component SHARED src/pre_approach_component.cpp)
ament_target_dependencies(pre_approach_component
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
rclcpp_components_register_nodes(pre_approach_component "my_components::PreApproach")

add_library(attach_server_component SHARED src/attach_server_component.cpp)
ament_target_dependencies(attach_server_component
  attach_shelf
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
rclcpp_components_register_nodes(attach_server_component "my_components::AttachServer")

add_library(attach_client_component SHARED src/attach_client_component.cpp)
ament_target_dependencies(attach_client_component
  attach_shelf
  rclcpp
  rclcpp_components
)
rclcpp_components_register_nodes(attach_client_component "my_components::AttachClient")

install(TARGETS
  pre_approach_component
  attach_server_component
  attach_client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
