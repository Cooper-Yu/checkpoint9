cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
)

add_executable(pre_approach src/pre_approach.cpp)
ament_target_dependencies(pre_approach
  rclcpp
  geometry_msgs
  sensor_msgs
)

add_executable(approach_service_server src/approach_service_server.cpp)
ament_target_dependencies(approach_service_server
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2_ros
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(approach_service_server "${cpp_typesupport_target}")

install(TARGETS
  pre_approach
  approach_service_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
